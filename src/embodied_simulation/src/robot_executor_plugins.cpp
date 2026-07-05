#include "embodied_simulation/robot_executor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory>
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

class GazeboRobotExecutor : public RobotExecutor
{
public:
  void configure(const ControllerConfig & config) override
  {
    controller_ = std::make_unique<SimulationController>(config);
  }

  bool execute(const RobotCommand & command, double now_s) override
  {
    if (!controller_) {
      return false;
    }
    if (command.action_type == RobotCommand::MOVE) {
      // MOVE 同时支持 linear_x 与 angular_z，因此“绕圈/画圆”无需新增接口字段。
      controller_->set_manual_command(
        command.linear_x, command.angular_z, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::TURN) {
      controller_->set_manual_command(
        0.0, command.angular_z, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::STOP) {
      controller_->stop();
      return true;
    }
    if (command.action_type == RobotCommand::NAVIGATE_TO) {
      // v0.4 的仿真导航先模拟 Nav2 长动作窗口：真实目标点坐标由后续 Nav2 bridge
      // 解析 places.yaml；这里保持 /cmd_vel 可观测，保证语音→导航 action 链路可验收。
      controller_->set_manual_command(0.14, 0.0, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::FOLLOW_WAYPOINTS) {
      controller_->set_manual_command(0.12, 0.25, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::CANCEL_NAVIGATION) {
      controller_->stop();
      return true;
    }
    if (command.action_type == RobotCommand::SET_MODE) {
      return controller_->set_mode(command.mode);
    }
    if (command.action_type == RobotCommand::WAVE ||
      command.action_type == RobotCommand::SET_LED)
    {
      // 仿真环境没有真实机械臂/灯带，这类附件动作只 ACK 并停车，保留接口可演示扩展点。
      controller_->stop();
      return true;
    }
    return false;
  }

  void stop() override
  {
    if (controller_) {
      controller_->stop();
    }
  }

  void update_scan(
    const std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max,
    double now_s) override
  {
    if (controller_) {
      controller_->update_scan(
        ranges, angle_min, angle_increment, range_min, range_max, now_s);
    }
  }

  ControllerOutput step(double now_s) override
  {
    return controller_ ? controller_->step(now_s) : ControllerOutput{};
  }

  std::string mode_name() const override
  {
    return controller_ ? SimulationController::mode_name(controller_->mode()) : "manual";
  }

  std::string backend_name() const override {return "simulation";}

private:
  std::unique_ptr<SimulationController> controller_;
};

class MockRobotExecutor : public RobotExecutor
{
public:
  void configure(const ControllerConfig & config) override
  {
    config_ = config;
    stop();
  }

  bool execute(const RobotCommand & command, double now_s) override
  {
    if (command.action_type == RobotCommand::MOVE) {
      mode_ = ControlMode::kManual;
      velocity_.linear_x = std::clamp(
        command.linear_x, -config_.max_linear_speed, config_.max_linear_speed);
      velocity_.angular_z = std::clamp(
        command.angular_z, -config_.max_angular_speed, config_.max_angular_speed);
      active_until_s_ = now_s + std::clamp(command.duration_s, 0.0, 10.0);
      return true;
    }
    if (command.action_type == RobotCommand::TURN) {
      mode_ = ControlMode::kManual;
      velocity_.linear_x = 0.0;
      velocity_.angular_z = std::clamp(
        command.angular_z, -config_.max_angular_speed, config_.max_angular_speed);
      active_until_s_ = now_s + std::clamp(command.duration_s, 0.0, 10.0);
      return true;
    }
    if (command.action_type == RobotCommand::STOP) {
      stop();
      return true;
    }
    if (command.action_type == RobotCommand::NAVIGATE_TO) {
      mode_ = ControlMode::kManual;
      velocity_.linear_x = 0.14;
      velocity_.angular_z = 0.0;
      active_until_s_ = now_s + std::clamp(command.duration_s, 0.0, 10.0);
      return true;
    }
    if (command.action_type == RobotCommand::FOLLOW_WAYPOINTS) {
      mode_ = ControlMode::kManual;
      velocity_.linear_x = 0.12;
      velocity_.angular_z = 0.25;
      active_until_s_ = now_s + std::clamp(command.duration_s, 0.0, 10.0);
      return true;
    }
    if (command.action_type == RobotCommand::CANCEL_NAVIGATION) {
      stop();
      return true;
    }
    if (command.action_type == RobotCommand::SET_MODE) {
      if (command.mode == "manual") {
        mode_ = ControlMode::kManual;
      } else if (command.mode == "obstacle_avoidance") {
        mode_ = ControlMode::kObstacleAvoidance;
      } else if (command.mode == "wall_following") {
        mode_ = ControlMode::kWallFollowing;
      } else {
        return false;
      }
      velocity_ = {};
      return true;
    }
    if (command.action_type == RobotCommand::WAVE ||
      command.action_type == RobotCommand::SET_LED)
    {
      velocity_ = {};
      active_until_s_ = now_s;
      return true;
    }
    return false;
  }

  void stop() override
  {
    mode_ = ControlMode::kManual;
    velocity_ = {};
    active_until_s_ = 0.0;
  }

  void update_scan(
    const std::vector<float> &,
    double, double, double, double, double) override {}

  ControllerOutput step(double now_s) override
  {
    if (now_s > active_until_s_) {
      velocity_ = {};
    }
    ControllerOutput output;
    output.velocity = velocity_;
    output.mode = mode_;
    output.sensor_stale = false;
    output.front_distance = std::numeric_limits<double>::infinity();
    output.right_distance = std::numeric_limits<double>::infinity();
    output.reason = (velocity_.linear_x != 0.0 || velocity_.angular_z != 0.0) ?
      "mock_execution" : "mock_idle";
    return output;
  }

  std::string mode_name() const override
  {
    return SimulationController::mode_name(mode_);
  }

  std::string backend_name() const override {return "mock";}

private:
  ControllerConfig config_;
  ControlMode mode_{ControlMode::kManual};
  VelocityCommand velocity_;
  double active_until_s_{0.0};
};

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
    bool canceled = false;
    if (navigate_goal_handle_) {
      navigate_client_->async_cancel_goal(navigate_goal_handle_);
      navigate_goal_handle_.reset();
      canceled = true;
    }
    if (follow_goal_handle_) {
      follow_client_->async_cancel_goal(follow_goal_handle_);
      follow_goal_handle_.reset();
      canceled = true;
    }
    active_.store(false);
    if (canceled) {
      set_external_state(ActionExecutionState::kCanceled);
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

  std::string mode_name() const override
  {
    return active_.load() ? "nav2_navigation" : "manual";
  }

  std::string backend_name() const override {return "nav2";}

private:
  bool send_navigate_goal(const std::string & target)
  {
    if (!navigate_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
      return false;
    }
    set_external_state(ActionExecutionState::kRunning);
    NavigateToPose::Goal goal;
    goal.pose = places_.to_pose_stamped(target);
    goal.pose.header.stamp = node_->now();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const NavigateGoalHandle::SharedPtr & handle) {
        navigate_goal_handle_ = handle;
        active_.store(handle != nullptr);
      };
    options.result_callback =
      [this](const NavigateGoalHandle::WrappedResult & result) {
        navigate_goal_handle_.reset();
        active_.store(false);
        set_external_state(state_from_result_code(result.code));
      };
    auto future = navigate_client_->async_send_goal(goal, options);
    const bool accepted =
      future.wait_for(std::chrono::seconds(1)) == std::future_status::ready &&
      future.get() != nullptr;
    if (!accepted) {
      set_external_state(ActionExecutionState::kBlocked);
      active_.store(false);
    }
    return accepted;
  }

  bool send_follow_goal(
    const std::vector<std::string> & waypoints,
    std::uint32_t loops)
  {
    if (!follow_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
      return false;
    }
    set_external_state(ActionExecutionState::kRunning);
    FollowWaypoints::Goal goal;
    goal.number_of_loops = std::max<std::uint32_t>(1U, loops);
    for (const auto & waypoint : waypoints) {
      auto pose = places_.to_pose_stamped(waypoint);
      pose.header.stamp = node_->now();
      goal.poses.push_back(pose);
    }
    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const FollowGoalHandle::SharedPtr & handle) {
        follow_goal_handle_ = handle;
        active_.store(handle != nullptr);
      };
    options.result_callback =
      [this](const FollowGoalHandle::WrappedResult & result) {
        follow_goal_handle_.reset();
        active_.store(false);
        set_external_state(state_from_result_code(result.code));
      };
    auto future = follow_client_->async_send_goal(goal, options);
    const bool accepted =
      future.wait_for(std::chrono::seconds(1)) == std::future_status::ready &&
      future.get() != nullptr;
    if (!accepted) {
      set_external_state(ActionExecutionState::kBlocked);
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

  void set_external_state(ActionExecutionState state)
  {
    external_state_.store(static_cast<int>(state));
  }

  Nav2Places places_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_client_;
  NavigateGoalHandle::SharedPtr navigate_goal_handle_;
  FollowGoalHandle::SharedPtr follow_goal_handle_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool active_{false};
  std::atomic<int> external_state_{
    static_cast<int>(ActionExecutionState::kSucceeded)};
};

}  // namespace embodied_simulation

PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::GazeboRobotExecutor,
  embodied_simulation::RobotExecutor)
PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::MockRobotExecutor,
  embodied_simulation::RobotExecutor)
PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::Nav2RobotExecutor,
  embodied_simulation::RobotExecutor)
