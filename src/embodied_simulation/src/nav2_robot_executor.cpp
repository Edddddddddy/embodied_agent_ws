#include "embodied_simulation/robot_executor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "embodied_simulation/nav2_places.hpp"

namespace embodied_simulation
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

class Nav2RobotExecutor : public RobotExecutor
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  ~Nav2RobotExecutor() override
  {
    stop();
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void configure(const ControllerConfig &) override
  {
    const char * configured_path = std::getenv("EMBODIED_NAV2_PLACES_FILE");
    const std::string places_path = configured_path && configured_path[0] != '\0' ?
      std::string(configured_path) :
      ament_index_cpp::get_package_share_directory("embodied_simulation") +
      "/config/places.yaml";
    places_ = load_nav2_places_yaml(places_path);
    node_ = std::make_shared<rclcpp::Node>("nav2_robot_executor");
    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(
      node_, "navigate_to_pose");
    follow_client_ = rclcpp_action::create_client<FollowWaypoints>(
      node_, "follow_waypoints");
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
  }

  bool execute(const RobotCommand & command, double) override
  {
    if (!node_) {
      return false;
    }
    if (command.action_type == RobotCommand::NAVIGATE_TO) {
      return send_navigate_goal(command.target);
    }
    if (command.action_type == RobotCommand::FOLLOW_WAYPOINTS) {
      return send_follow_goal(command.waypoints, command.number_of_loops);
    }
    if (command.action_type == RobotCommand::CANCEL_NAVIGATION ||
      command.action_type == RobotCommand::STOP)
    {
      stop();
      return true;
    }
    return false;
  }

  void stop() override
  {
    NavigateGoalHandle::SharedPtr navigate_handle;
    FollowGoalHandle::SharedPtr follow_handle;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      navigate_handle = navigate_goal_handle_;
      follow_handle = follow_goal_handle_;
      navigate_goal_handle_.reset();
      follow_goal_handle_.reset();
      // 递增代次后，晚到的旧 result callback 只能被丢弃，不能覆盖下一条
      // Nav2 goal 的 active/detail 状态。
      ++navigate_generation_;
      ++follow_generation_;
    }
    bool canceled = false;
    if (navigate_handle) {
      navigate_client_->async_cancel_goal(navigate_handle);
      canceled = true;
    }
    if (follow_handle) {
      follow_client_->async_cancel_goal(follow_handle);
      canceled = true;
    }
    active_.store(false);
    if (canceled) {
      set_external_state(ActionExecutionState::kCanceled, "nav2:cancel_requested");
    }
  }

  void update_scan(
    const std::vector<float> &, double, double, double, double, double) override {}

  ControllerOutput step(double) override
  {
    ControllerOutput output;
    output.mode = ControlMode::kManual;
    output.sensor_stale = false;
    output.reason = active_.load() ? "nav2_goal_active" : "nav2_idle";
    return output;
  }

  bool publishes_cmd_vel() const override {return false;}

  std::optional<ActionExecutionUpdate> external_action_update() const override
  {
    return ActionExecutionUpdate{
      static_cast<ActionExecutionState>(external_state_.load()),
      active_.load() ? 0.5 : 1.0};
  }

  std::string external_action_detail() const override
  {
    std::lock_guard<std::mutex> lock(detail_mutex_);
    return external_detail_;
  }

  std::string mode_name() const override
  {
    return active_.load() ? "nav2_navigation" : "manual";
  }

  std::string backend_name() const override {return "nav2";}

private:
  bool send_navigate_goal(const std::string & target)
  {
    if (!navigate_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:navigate_to_pose:server_unavailable target=" + target);
      return false;
    }
    set_external_state(
      ActionExecutionState::kRunning,
      "nav2:navigate_to_pose:sending target=" + target);
    NavigateToPose::Goal goal;
    try {
      goal.pose = places_.to_pose_stamped(target);
    } catch (const std::out_of_range &) {
      // ActionGuard 白名单与部署现场 places 文件可能发生配置漂移；执行器必须
      // 返回可观测的 blocked 结果，不能让 plugin 异常杀死组件容器。
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:navigate_to_pose:unknown_target target=" + target);
      return false;
    }
    goal.pose.header.stamp = node_->now();
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      generation = ++navigate_generation_;
      navigate_goal_handle_.reset();
    }
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, target, generation](const NavigateGoalHandle::SharedPtr & handle) {
        bool stale = false;
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          stale = generation != navigate_generation_;
          if (!stale) {
            navigate_goal_handle_ = handle;
          }
        }
        if (stale) {
          // stop() 可能发生在 goal response 到达之前；此时仍要取消刚被 Nav2
          // 接受的旧 goal，不能只忽略回调而留下后台“幽灵导航”。
          if (handle) {
            navigate_client_->async_cancel_goal(handle);
          }
          return;
        }
        active_.store(handle != nullptr);
        if (handle) {
          set_external_state(
            ActionExecutionState::kRunning,
            "nav2:navigate_to_pose:accepted target=" + target);
        } else {
          set_external_state(
            ActionExecutionState::kBlocked,
            "nav2:navigate_to_pose:goal_rejected target=" + target);
        }
      };
    options.result_callback =
      [this, target, generation](const NavigateGoalHandle::WrappedResult & result) {
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (generation != navigate_generation_) {
            return;
          }
          navigate_goal_handle_.reset();
        }
        active_.store(false);
        set_external_state(
          state_from_result_code(result.code),
          navigate_result_detail(target, result));
      };
    auto future = navigate_client_->async_send_goal(goal, options);
    const bool accepted =
      future.wait_for(std::chrono::seconds(1)) == std::future_status::ready &&
      future.get() != nullptr;
    if (!accepted) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:navigate_to_pose:goal_response_timeout_or_rejected target=" + target);
      active_.store(false);
    }
    return accepted;
  }

  bool send_follow_goal(
    const std::vector<std::string> & waypoints,
    std::uint32_t loops)
  {
    if (!follow_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:follow_waypoints:server_unavailable waypoints=" + join(waypoints, ","));
      return false;
    }
    set_external_state(
      ActionExecutionState::kRunning,
      "nav2:follow_waypoints:sending waypoints=" + join(waypoints, ","));
    FollowWaypoints::Goal goal;
    goal.number_of_loops = std::max<std::uint32_t>(1U, loops);
    try {
      for (const auto & waypoint : waypoints) {
        auto pose = places_.to_pose_stamped(waypoint);
        pose.header.stamp = node_->now();
        goal.poses.push_back(pose);
      }
    } catch (const std::out_of_range &) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:follow_waypoints:unknown_waypoint waypoints=" + join(waypoints, ","));
      return false;
    }
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      generation = ++follow_generation_;
      follow_goal_handle_.reset();
    }
    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
    options.goal_response_callback =
      [this, waypoints, generation](const FollowGoalHandle::SharedPtr & handle) {
        bool stale = false;
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          stale = generation != follow_generation_;
          if (!stale) {
            follow_goal_handle_ = handle;
          }
        }
        if (stale) {
          if (handle) {
            follow_client_->async_cancel_goal(handle);
          }
          return;
        }
        active_.store(handle != nullptr);
        if (handle) {
          set_external_state(
            ActionExecutionState::kRunning,
            "nav2:follow_waypoints:accepted waypoints=" + join(waypoints, ","));
        } else {
          set_external_state(
            ActionExecutionState::kBlocked,
            "nav2:follow_waypoints:goal_rejected waypoints=" + join(waypoints, ","));
        }
      };
    options.result_callback =
      [this, waypoints, generation](const FollowGoalHandle::WrappedResult & result) {
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (generation != follow_generation_) {
            return;
          }
          follow_goal_handle_.reset();
        }
        active_.store(false);
        set_external_state(
          state_from_result_code(result.code),
          follow_result_detail(waypoints, result));
      };
    auto future = follow_client_->async_send_goal(goal, options);
    const bool accepted =
      future.wait_for(std::chrono::seconds(1)) == std::future_status::ready &&
      future.get() != nullptr;
    if (!accepted) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:follow_waypoints:goal_response_timeout_or_rejected waypoints=" +
        join(waypoints, ","));
      active_.store(false);
    }
    return accepted;
  }

  static ActionExecutionState state_from_result_code(rclcpp_action::ResultCode code)
  {
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      return ActionExecutionState::kSucceeded;
    }
    if (code == rclcpp_action::ResultCode::CANCELED) {
      return ActionExecutionState::kCanceled;
    }
    return ActionExecutionState::kBlocked;
  }

  static std::string result_code_name(rclcpp_action::ResultCode code)
  {
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      return "succeeded";
    }
    if (code == rclcpp_action::ResultCode::CANCELED) {
      return "canceled";
    }
    if (code == rclcpp_action::ResultCode::ABORTED) {
      return "aborted";
    }
    return "unknown";
  }

  static std::string join(const std::vector<std::string> & values, const std::string & separator)
  {
    std::ostringstream stream;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        stream << separator;
      }
      stream << values[index];
    }
    return stream.str();
  }

  static std::string navigate_result_detail(
    const std::string & target,
    const NavigateGoalHandle::WrappedResult & result)
  {
    std::ostringstream stream;
    stream << "nav2:navigate_to_pose:" << result_code_name(result.code)
           << " target=" << target;
    if (result.result) {
      stream << " error_code=" << result.result->error_code;
      if (!result.result->error_msg.empty()) {
        stream << " error_msg=" << result.result->error_msg;
      }
    }
    return stream.str();
  }

  static std::string follow_result_detail(
    const std::vector<std::string> & waypoints,
    const FollowGoalHandle::WrappedResult & result)
  {
    std::ostringstream stream;
    stream << "nav2:follow_waypoints:" << result_code_name(result.code)
           << " waypoints=" << join(waypoints, ",");
    if (result.result) {
      stream << " error_code=" << result.result->error_code
             << " missed_waypoints=" << result.result->missed_waypoints.size();
      if (!result.result->error_msg.empty()) {
        stream << " error_msg=" << result.result->error_msg;
      }
    }
    return stream.str();
  }

  void set_external_state(ActionExecutionState state, const std::string & detail)
  {
    external_state_.store(static_cast<int>(state));
    std::lock_guard<std::mutex> lock(detail_mutex_);
    external_detail_ = detail;
  }

  Nav2Places places_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client_;
  NavigateGoalHandle::SharedPtr navigate_goal_handle_;
  FollowGoalHandle::SharedPtr follow_goal_handle_;
  std::mutex goal_mutex_;
  std::uint64_t navigate_generation_{0};
  std::uint64_t follow_generation_{0};
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool active_{false};
  std::atomic<int> external_state_{
    static_cast<int>(ActionExecutionState::kSucceeded)};
  mutable std::mutex detail_mutex_;
  std::string external_detail_{"nav2:idle"};
};

}  // namespace embodied_simulation

PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::Nav2RobotExecutor,
  embodied_simulation::RobotExecutor)
