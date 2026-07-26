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
#include "embodied_simulation/nav2_result_policy.hpp"

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
  using NavigateCancelResponse =
    rclcpp_action::Client<NavigateToPose>::CancelResponse;
  using FollowCancelResponse =
    rclcpp_action::Client<FollowWaypoints>::CancelResponse;

  ~Nav2RobotExecutor() override
  {
    request_stop(StopClock::now());
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void configure(const ControllerConfig & config) override
  {
    stop_timeout_ = std::chrono::duration_cast<StopClock::duration>(
      std::chrono::duration<double>(config.stop_timeout_s));
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
      request_stop(StopClock::now());
      return true;
    }
    return false;
  }

  void request_stop(StopTimePoint requested_at) override
  {
    NavigateGoalHandle::SharedPtr navigate_handle;
    FollowGoalHandle::SharedPtr follow_handle;
    std::uint64_t navigate_generation = 0;
    std::uint64_t follow_generation = 0;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (stop_state_ == StopExecutionState::kStopping ||
        ((stop_state_ == StopExecutionState::kFailed ||
        stop_state_ == StopExecutionState::kTimedOut) &&
        has_unsettled_goal_locked()))
      {
        // STOP 是幂等操作；等待同一个底层 goal 的 terminal，不能重复发取消并
        // 用新的 deadline 掩盖第一次超时。
        return;
      }
      navigate_handle = navigate_goal_handle_;
      follow_handle = follow_goal_handle_;
      navigate_generation = navigate_generation_;
      follow_generation = follow_generation_;
      stop_requested_ = true;
      stop_deadline_ = requested_at + stop_timeout_;
      if (!has_unsettled_goal_locked()) {
        mark_quiesced_locked("nav2:stop:no_active_goal");
        active_.store(false);
        return;
      }
      stop_state_ = StopExecutionState::kStopping;
      stop_detail_ = "nav2:stop:waiting_terminal";
    }
    set_external_state(ActionExecutionState::kRunning, "nav2:stop:waiting_terminal");
    if (navigate_handle) {
      request_navigate_cancel(navigate_handle, navigate_generation);
    }
    if (follow_handle) {
      request_follow_cancel(follow_handle, follow_generation);
    }
    // goal response 可能尚未到达；response callback 会在保存 handle 后补发取消。
  }

  StopExecutionUpdate poll_stop(StopTimePoint now) override
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (stop_state_ == StopExecutionState::kStopping && now >= stop_deadline_) {
      stop_state_ = StopExecutionState::kTimedOut;
      stop_detail_ = "nav2:stop:terminal_timeout";
    }
    return {stop_state_, stop_detail_};
  }

  bool is_quiesced() const override
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    return !has_unsettled_goal_locked() &&
           stop_state_ == StopExecutionState::kQuiesced;
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
      if (has_unsettled_goal_locked()) {
        set_external_state(
          ActionExecutionState::kBlocked,
          "nav2:navigate_to_pose:previous_goal_not_quiesced target=" + target);
        return false;
      }
      generation = ++navigate_generation_;
      navigate_goal_handle_.reset();
      navigate_goal_pending_ = true;
      stop_requested_ = false;
      stop_state_ = StopExecutionState::kQuiesced;
      stop_detail_ = "nav2:goal_active";
    }
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, target, generation](const NavigateGoalHandle::SharedPtr & handle) {
        bool stale = false;
        bool stop_requested = false;
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          stale = generation != navigate_generation_;
          if (!stale) {
            navigate_goal_pending_ = false;
            navigate_goal_handle_ = handle;
            stop_requested = stop_requested_;
            if (!handle && stop_requested_ && !has_unsettled_goal_locked()) {
              mark_quiesced_locked("nav2:stop:goal_rejected_before_execution");
            }
          }
        }
        if (stale) {
          // goal response 晚到时仍要取消对应 goal，不能只忽略回调而留下
          // 后台“幽灵导航”。这里不把 cancel ACK 当作 terminal。
          if (handle) {
            navigate_client_->async_cancel_goal(handle);
          }
          return;
        }
        active_.store(handle != nullptr);
        if (handle) {
          if (stop_requested) {
            request_navigate_cancel(handle, generation);
          } else {
            set_external_state(
              ActionExecutionState::kRunning,
              "nav2:navigate_to_pose:accepted target=" + target);
          }
        } else {
          set_external_state(
            ActionExecutionState::kBlocked,
            "nav2:navigate_to_pose:goal_rejected target=" + target);
        }
      };
    options.result_callback =
      [this, target, generation](const NavigateGoalHandle::WrappedResult & result) {
        const auto detail = navigate_result_detail(target, result);
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (generation != navigate_generation_) {
            return;
          }
          navigate_goal_pending_ = false;
          navigate_goal_handle_.reset();
          if (stop_requested_ && !has_unsettled_goal_locked()) {
            // 取消响应仅表示 server 收到了请求；只有 result callback 才证明
            // NavigateToPose 已进入 CANCELED/ABORTED/SUCCEEDED terminal。
            mark_quiesced_locked("nav2:stop:terminal " + detail);
          }
        }
        active_.store(false);
        set_external_state(state_from_result_code(result.code), detail);
      };
    auto future = navigate_client_->async_send_goal(goal, options);
    const auto response_status = future.wait_for(std::chrono::seconds(1));
    const bool accepted =
      response_status == std::future_status::ready && future.get() != nullptr;
    if (!accepted) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:navigate_to_pose:goal_response_timeout_or_rejected target=" + target);
      active_.store(false);
      if (response_status != std::future_status::ready) {
        // 请求可能已经被 server 接收，只是 response 超时；保持 pending 并进入
        // fail-closed 停止流程，晚到的 response callback 会补发取消。
        request_stop(StopClock::now());
      } else {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (generation == navigate_generation_) {
          navigate_goal_pending_ = false;
        }
      }
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
    // RobotCommand 约定的是“总遍历轮数”，而 Nav2 FollowWaypoints 约定的是
    // “首轮之后额外重复几次”。因此 1 轮必须下发 0，避免巡检路线多走一遍。
    const auto total_traversals = std::max<std::uint32_t>(1U, loops);
    goal.number_of_loops = total_traversals - 1U;
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
      if (has_unsettled_goal_locked()) {
        set_external_state(
          ActionExecutionState::kBlocked,
          "nav2:follow_waypoints:previous_goal_not_quiesced waypoints=" +
          join(waypoints, ","));
        return false;
      }
      generation = ++follow_generation_;
      follow_goal_handle_.reset();
      follow_goal_pending_ = true;
      stop_requested_ = false;
      stop_state_ = StopExecutionState::kQuiesced;
      stop_detail_ = "nav2:goal_active";
    }
    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
    options.goal_response_callback =
      [this, waypoints, generation](const FollowGoalHandle::SharedPtr & handle) {
        bool stale = false;
        bool stop_requested = false;
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          stale = generation != follow_generation_;
          if (!stale) {
            follow_goal_pending_ = false;
            follow_goal_handle_ = handle;
            stop_requested = stop_requested_;
            if (!handle && stop_requested_ && !has_unsettled_goal_locked()) {
              mark_quiesced_locked("nav2:stop:goal_rejected_before_execution");
            }
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
          if (stop_requested) {
            request_follow_cancel(handle, generation);
          } else {
            set_external_state(
              ActionExecutionState::kRunning,
              "nav2:follow_waypoints:accepted waypoints=" + join(waypoints, ","));
          }
        } else {
          set_external_state(
            ActionExecutionState::kBlocked,
            "nav2:follow_waypoints:goal_rejected waypoints=" + join(waypoints, ","));
        }
      };
    options.result_callback =
      [this, waypoints, generation](const FollowGoalHandle::WrappedResult & result) {
        const auto outcome = evaluate_follow_waypoints_result(
          waypoints, result.code, result.result.get());
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (generation != follow_generation_) {
            return;
          }
          follow_goal_pending_ = false;
          follow_goal_handle_.reset();
          if (stop_requested_ && !has_unsettled_goal_locked()) {
            mark_quiesced_locked("nav2:stop:terminal " + outcome.detail);
          }
        }
        active_.store(false);
        set_external_state(outcome.state, outcome.detail);
      };
    auto future = follow_client_->async_send_goal(goal, options);
    const auto response_status = future.wait_for(std::chrono::seconds(1));
    const bool accepted =
      response_status == std::future_status::ready && future.get() != nullptr;
    if (!accepted) {
      set_external_state(
        ActionExecutionState::kBlocked,
        "nav2:follow_waypoints:goal_response_timeout_or_rejected waypoints=" +
        join(waypoints, ","));
      active_.store(false);
      if (response_status != std::future_status::ready) {
        request_stop(StopClock::now());
      } else {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (generation == follow_generation_) {
          follow_goal_pending_ = false;
        }
      }
    }
    return accepted;
  }

  bool has_unsettled_goal_locked() const
  {
    return navigate_goal_pending_ || follow_goal_pending_ ||
           navigate_goal_handle_ || follow_goal_handle_;
  }

  void mark_quiesced_locked(const std::string & detail)
  {
    stop_requested_ = false;
    stop_state_ = StopExecutionState::kQuiesced;
    stop_detail_ = detail;
  }

  void request_navigate_cancel(
    const NavigateGoalHandle::SharedPtr & handle,
    std::uint64_t generation)
  {
    try {
      navigate_client_->async_cancel_goal(
        handle,
        [this, generation](const NavigateCancelResponse::SharedPtr & response) {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (generation != navigate_generation_ || !stop_requested_ ||
            stop_state_ != StopExecutionState::kStopping)
          {
            return;
          }
          if (!response ||
            response->return_code != NavigateCancelResponse::ERROR_NONE ||
            response->goals_canceling.empty())
          {
            // cancel response 只表示请求是否被接受。拒绝时仍保留 goal handle，
            // 禁止后续动作抢占底盘；只有真正的 result callback 才可解除封锁。
            stop_state_ = StopExecutionState::kFailed;
            stop_detail_ = "nav2:stop:cancel_request_rejected";
          }
        });
    } catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (generation == navigate_generation_ && stop_requested_) {
        stop_state_ = StopExecutionState::kFailed;
        stop_detail_ = std::string("nav2:stop:cancel_request_failed error=") +
          error.what();
      }
    }
  }

  void request_follow_cancel(
    const FollowGoalHandle::SharedPtr & handle,
    std::uint64_t generation)
  {
    try {
      follow_client_->async_cancel_goal(
        handle,
        [this, generation](const FollowCancelResponse::SharedPtr & response) {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (generation != follow_generation_ || !stop_requested_ ||
            stop_state_ != StopExecutionState::kStopping)
          {
            return;
          }
          if (!response ||
            response->return_code != FollowCancelResponse::ERROR_NONE ||
            response->goals_canceling.empty())
          {
            stop_state_ = StopExecutionState::kFailed;
            stop_detail_ = "nav2:stop:cancel_request_rejected";
          }
        });
    } catch (const std::exception & error) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (generation == follow_generation_ && stop_requested_) {
        stop_state_ = StopExecutionState::kFailed;
        stop_detail_ = std::string("nav2:stop:cancel_request_failed error=") +
          error.what();
      }
    }
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
  mutable std::mutex goal_mutex_;
  bool navigate_goal_pending_{false};
  bool follow_goal_pending_{false};
  std::uint64_t navigate_generation_{0};
  std::uint64_t follow_generation_{0};
  bool stop_requested_{false};
  StopExecutionState stop_state_{StopExecutionState::kQuiesced};
  StopTimePoint stop_deadline_{};
  StopClock::duration stop_timeout_{std::chrono::seconds(3)};
  std::string stop_detail_{"nav2:idle"};
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
