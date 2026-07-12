#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include <embodied_agent_interfaces/action/execute_robot_command.hpp>
#include <embodied_agent_interfaces/msg/behavior_tree_status.hpp>
#include <embodied_agent_interfaces/msg/component_health.hpp>
#include <embodied_agent_interfaces/msg/robot_action_ack.hpp>
#include <embodied_agent_interfaces/msg/simulation_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "embodied_simulation/active_action_runtime.hpp"
#include "embodied_simulation/command_behavior_tree.hpp"
#include "embodied_simulation/executor_diagnostics.hpp"
#include "embodied_simulation/node_configuration.hpp"
#include "embodied_simulation/robot_executor.hpp"
#include "embodied_simulation/simulation_controller.hpp"
#include "embodied_simulation/simulation_control_factory.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_simulation
{

class SimulationControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecuteRobotCommand =
    embodied_agent_interfaces::action::ExecuteRobotCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteRobotCommand>;
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SimulationControlNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("simulation_control", "", options),
    executor_loader_("embodied_simulation", "embodied_simulation::RobotExecutor")
  {
    RCLCPP_INFO(get_logger(), "simulation control lifecycle node created");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    using std::placeholders::_1;
    const auto controller_config = load_config();
    executor_plugin_ = string_parameter(
      "executor_plugin", "embodied_simulation/GazeboRobotExecutor");
    action_timeout_s_ = double_parameter("action_timeout_s", 12.0);
    const auto validation = validate_node_configuration(
      controller_config, get_parameter("control_rate_hz").as_double(),
      action_timeout_s_, executor_plugin_);
    if (!validation.valid) {
      RCLCPP_ERROR(
        get_logger(), "invalid simulation configuration: %s",
        validation.error.c_str());
      return CallbackReturn::FAILURE;
    }
    try {
      executor_ = executor_loader_.createSharedInstance(executor_plugin_);
      executor_->configure(controller_config);
      executor_backend_ = executor_->backend_name();
    } catch (const pluginlib::PluginlibException & error) {
      RCLCPP_ERROR(
        get_logger(), "failed to load executor plugin %s: %s",
        executor_plugin_.c_str(), error.what());
      return CallbackReturn::FAILURE;
    }
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", embodied_agent_middleware::command_qos(10));
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/control_mode", embodied_agent_middleware::state_qos());
    state_pub_ = create_publisher<embodied_agent_interfaces::msg::SimulationState>(
      "robot/simulation_state", embodied_agent_middleware::state_qos());
    action_ack_pub_ = create_publisher<embodied_agent_interfaces::msg::RobotActionAck>(
      "robot/action_ack", embodied_agent_middleware::event_qos());
    bt_status_pub_ = create_publisher<embodied_agent_interfaces::msg::BehaviorTreeStatus>(
      "robot/bt_status", embodied_agent_middleware::event_qos());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", embodied_agent_middleware::diagnostics_qos());
    health_pub_ = create_publisher<embodied_agent_interfaces::msg::ComponentHealth>(
      "system/component_health", embodied_agent_middleware::state_qos());
    use_behavior_tree_ = bool_parameter("use_behavior_tree", true);
    if (use_behavior_tree_) {
      const auto default_tree =
        ament_index_cpp::get_package_share_directory("embodied_simulation") +
        "/config/command_tree.xml";
      const auto tree_path = string_parameter("bt_xml_path", default_tree);
      std::ifstream stream(tree_path);
      if (!stream) {
        RCLCPP_ERROR(get_logger(), "failed to open BT XML: %s", tree_path.c_str());
        return CallbackReturn::FAILURE;
      }
      std::ostringstream xml;
      xml << stream.rdbuf();
      try {
        behavior_tree_ = std::make_unique<CommandBehaviorTree>(xml.str());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "failed to load BT XML: %s", error.what());
        return CallbackReturn::FAILURE;
      }
    }
    // ROS 节点只保留通信和 Lifecycle 职责；动作进度、超时、取消及 BT 终态
    // 统一交给可独立测试的运行时，避免 Nav2/定时动作各自维护一套状态机。
    action_runtime_ = std::make_unique<ActiveActionRuntime>(behavior_tree_.get());
    action_server_ = rclcpp_action::create_server<ExecuteRobotCommand>(
      this,
      "robot/execute_command",
      std::bind(
        &SimulationControlNode::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &SimulationControlNode::handle_cancel, this,
        std::placeholders::_1),
      std::bind(
        &SimulationControlNode::handle_accepted, this,
        std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "robot/control_mode_request", embodied_agent_middleware::command_qos(10),
      std::bind(&SimulationControlNode::on_mode_request, this, _1));
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
      "robot/emergency_stop", embodied_agent_middleware::command_qos(10),
      std::bind(&SimulationControlNode::on_emergency_stop, this, _1));

    const auto scan_qos = embodied_agent_middleware::sensor_qos();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", scan_qos,
      std::bind(&SimulationControlNode::on_scan, this, _1));

    const auto period = std::chrono::duration<double>(
      1.0 / get_parameter("control_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimulationControlNode::control_tick, this));
    timer_->cancel();
    diagnostics_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&SimulationControlNode::publish_diagnostics, this),
      diagnostics_callback_group_);
    diagnostics_timer_->cancel();
    RCLCPP_INFO(get_logger(), "simulation control configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    cmd_vel_pub_->on_activate();
    mode_pub_->on_activate();
    state_pub_->on_activate();
    action_ack_pub_->on_activate();
    bt_status_pub_->on_activate();
    diagnostics_pub_->on_activate();
    health_pub_->on_activate();
    timer_->reset();
    diagnostics_timer_->reset();
    publish_mode();
    publish_health(
      embodied_agent_interfaces::msg::ComponentHealth::STATE_READY,
      "executor_ready:" + executor_backend_);
    RCLCPP_INFO(get_logger(), "simulation control activated; mode=manual");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (active_goal_) {
      if (action_runtime_) {
        if (const auto tree = action_runtime_->cancel_tree("deactivated")) {
          publish_bt_status(*tree);
        }
      }
      finish_active_action(ActionExecutionState::kCanceled, "deactivated");
    }
    if (executor_) {
      executor_->stop();
    }
    publish_zero_velocity();
    if (timer_) {
      timer_->cancel();
    }
    if (diagnostics_timer_) {
      diagnostics_timer_->cancel();
    }
    publish_health(
      embodied_agent_interfaces::msg::ComponentHealth::STATE_STOPPED,
      "lifecycle_deactivated");
    health_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    bt_status_pub_->on_deactivate();
    action_ack_pub_->on_deactivate();
    state_pub_->on_deactivate();
    mode_pub_->on_deactivate();
    cmd_vel_pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "simulation control deactivated and stopped");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    reset_interfaces();
    executor_.reset();
    RCLCPP_INFO(get_logger(), "simulation control cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    if (executor_) {
      executor_->stop();
    }
    reset_interfaces();
    executor_.reset();
    RCLCPP_INFO(get_logger(), "simulation control shut down");
    return CallbackReturn::SUCCESS;
  }

private:
  bool is_active() const
  {
    return get_current_state().id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  double double_parameter(const std::string & name, double default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter(name, default_value);
    }
    return get_parameter(name).as_double();
  }

  bool bool_parameter(const std::string & name, bool default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter(name, default_value);
    }
    return get_parameter(name).as_bool();
  }

  std::string string_parameter(
    const std::string & name, const std::string & default_value)
  {
    if (!has_parameter(name)) {
      declare_parameter(name, default_value);
    }
    return get_parameter(name).as_string();
  }

  ControllerConfig load_config()
  {
    ControllerConfig config;
    config.max_linear_speed = double_parameter("max_linear_speed", 0.22);
    config.max_angular_speed = double_parameter("max_angular_speed", 1.0);
    config.linear_acceleration = double_parameter("linear_acceleration", 0.5);
    config.angular_acceleration = double_parameter("angular_acceleration", 2.0);
    config.emergency_distance = double_parameter("emergency_distance", 0.22);
    config.obstacle_distance = double_parameter("obstacle_distance", 0.50);
    config.wall_target_distance = double_parameter("wall_target_distance", 0.40);
    config.autonomous_linear_speed = double_parameter("autonomous_linear_speed", 0.14);
    config.obstacle_turn_speed = double_parameter("obstacle_turn_speed", 0.65);
    config.scan_timeout = double_parameter("scan_timeout", 0.50);
    config.wall_kp = double_parameter("wall_kp", 1.8);
    config.wall_ki = double_parameter("wall_ki", 0.0);
    config.wall_kd = double_parameter("wall_kd", 0.15);
    double_parameter("control_rate_hz", 20.0);
    return config;
  }

  double now_seconds() const
  {
    return get_clock()->now().seconds();
  }

  static bool supported_action_goal(
    const embodied_agent_interfaces::msg::RobotCommand & command)
  {
    using Command = embodied_agent_interfaces::msg::RobotCommand;
    if (command.action_type == Command::STOP) {
      return true;
    }
    if (command.action_type == Command::SET_MODE) {
      return command.mode == "manual" ||
             command.mode == "obstacle_avoidance" ||
             command.mode == "wall_following";
    }
    if (command.action_type == Command::WAVE) {
      return command.count >= 1 && command.count <= 5;
    }
    if (command.action_type == Command::SET_LED) {
      return command.color == "off" ||
             command.color == "red" ||
             command.color == "green" ||
             command.color == "blue" ||
             command.color == "yellow" ||
             command.color == "white";
    }
    if (command.action_type == Command::MOVE) {
      return std::isfinite(command.linear_x) &&
             std::isfinite(command.angular_z) &&
             std::isfinite(command.duration_s) &&
             command.duration_s >= 0.0 && command.duration_s <= 10.0;
    }
    if (command.action_type == Command::TURN) {
      return std::isfinite(command.angular_z) &&
             std::isfinite(command.duration_s) &&
             command.duration_s >= 0.0 && command.duration_s <= 10.0;
    }
    if (command.action_type == Command::NAVIGATE_TO) {
      return !command.target.empty() &&
             std::isfinite(command.duration_s) &&
             command.duration_s >= 0.0 && command.duration_s <= 10.0;
    }
    if (command.action_type == Command::FOLLOW_WAYPOINTS) {
      return !command.waypoints.empty() &&
             command.number_of_loops >= 1 &&
             command.number_of_loops <= 3 &&
             std::isfinite(command.duration_s) &&
             command.duration_s >= 0.0 && command.duration_s <= 10.0;
    }
    if (command.action_type == Command::CANCEL_NAVIGATION) {
      return true;
    }
    return false;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteRobotCommand::Goal> goal)
  {
    if (!is_active()) {
      RCLCPP_WARN(get_logger(), "rejected typed action goal while inactive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!supported_action_goal(goal->command) && !behavior_tree_) {
      RCLCPP_WARN(get_logger(), "rejected unsupported typed action goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    return active_goal_ == goal_handle ?
           rclcpp_action::CancelResponse::ACCEPT :
           rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    if (active_goal_) {
      if (action_runtime_) {
        if (const auto tree = action_runtime_->cancel_tree("preempted_by_new_goal")) {
          publish_bt_status(*tree);
        }
      }
      finish_active_action(
        ActionExecutionState::kCanceled, "preempted_by_new_goal");
    }

    const auto & command = goal_handle->get_goal()->command;
    using Command = embodied_agent_interfaces::msg::RobotCommand;
    if (command.action_type == Command::STOP) {
      executor_->stop();
      finish_immediate_action(goal_handle, true, "stopped");
      publish_action_ack("stop", "accepted");
      return;
    }
    if (command.action_type == Command::CANCEL_NAVIGATION) {
      executor_->stop();
      finish_immediate_action(goal_handle, true, "navigation_canceled");
      publish_action_ack("cancel_navigation", "accepted");
      return;
    }
    if (command.action_type == Command::SET_MODE) {
      const bool accepted = set_mode(command.mode);
      finish_immediate_action(
        goal_handle, accepted,
        accepted ? "mode_changed" : "unsupported_mode");
      publish_action_ack("set_mode", accepted ? "accepted" : "rejected");
      return;
    }
    if (command.action_type == Command::WAVE) {
      const bool accepted = executor_->execute(command, now_seconds());
      finish_immediate_action(
        goal_handle, accepted,
        accepted ? "wave_acknowledged" : "executor_rejected");
      publish_action_ack("wave", accepted ? "accepted" : "rejected");
      return;
    }
    if (command.action_type == Command::SET_LED) {
      const bool accepted = executor_->execute(command, now_seconds());
      finish_immediate_action(
        goal_handle, accepted,
        accepted ? "led_acknowledged" : "executor_rejected");
      publish_action_ack("set_led", accepted ? "accepted" : "rejected");
      return;
    }

    if (behavior_tree_) {
      active_goal_ = goal_handle;
      action_active_ = true;
      behavior_tree_->start(command);
      const auto initial = behavior_tree_->tick(
        false, ActionExecutionState::kRunning, "accepted");
      publish_bt_status(initial);
      if (initial.outcome == CommandTreeOutcome::kRejected) {
        finish_immediate_action(goal_handle, false, initial.detail);
        publish_action_ack("unknown", "rejected", initial.detail);
        clear_pending_long_action();
        return;
      }
    }

    const double now = now_seconds();
    std::string action_name;
    if (command.action_type == Command::MOVE) {
      if (!executor_->execute(command, now)) {
        finish_immediate_action(goal_handle, false, "executor_rejected");
        clear_pending_long_action();
        return;
      }
      action_name = "move";
    } else if (command.action_type == Command::TURN) {
      if (!executor_->execute(command, now)) {
        finish_immediate_action(goal_handle, false, "executor_rejected");
        clear_pending_long_action();
        return;
      }
      action_name = "turn";
    } else if (command.action_type == Command::NAVIGATE_TO) {
      if (!executor_->execute(command, now)) {
        const auto detail = executor_->external_action_detail();
        finish_immediate_action(
          goal_handle, false, detail.empty() ? "executor_rejected" : detail);
        clear_pending_long_action();
        return;
      }
      action_name = "navigate_to";
    } else if (command.action_type == Command::FOLLOW_WAYPOINTS) {
      if (!executor_->execute(command, now)) {
        const auto detail = executor_->external_action_detail();
        finish_immediate_action(
          goal_handle, false, detail.empty() ? "executor_rejected" : detail);
        clear_pending_long_action();
        return;
      }
      action_name = "follow_waypoints";
    } else {
      finish_immediate_action(goal_handle, false, "invalid_command");
      clear_pending_long_action();
      return;
    }
    active_goal_ = goal_handle;
    action_active_ = true;
    const bool uses_external_result =
      (action_name == "navigate_to" || action_name == "follow_waypoints") &&
      executor_->external_action_update().has_value();
    const double execution_duration_s = uses_external_result ?
      action_timeout_s_ + 1.0 : command.duration_s;
    action_runtime_->start(
      action_name, execution_duration_s, action_timeout_s_, now,
      uses_external_result);
    publish_action_feedback(
      ExecuteRobotCommand::Feedback::PHASE_ACCEPTED, 0.0F, "accepted");
    publish_action_ack(action_name, "accepted");
  }

  void finish_immediate_action(
    const std::shared_ptr<GoalHandle> & goal_handle,
    bool success,
    const std::string & message)
  {
    auto result = std::make_shared<ExecuteRobotCommand::Result>();
    result->success = success;
    result->status = success ?
      ExecuteRobotCommand::Result::STATUS_SUCCEEDED :
      ExecuteRobotCommand::Result::STATUS_REJECTED;
    result->message = message;
    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void clear_pending_long_action()
  {
    // BT 校验发生在 executor 接受之前；启动失败时必须同时清掉诊断状态，
    // 否则 diagnostics 会长期误报 action_active=true。
    active_goal_.reset();
    action_active_ = false;
    if (action_runtime_) {
      action_runtime_->reset();
    }
  }

  void publish_action_feedback(
    std::uint8_t phase, float progress, const std::string & detail)
  {
    if (!active_goal_) {
      return;
    }
    auto feedback = std::make_shared<ExecuteRobotCommand::Feedback>();
    feedback->phase = phase;
    feedback->progress = progress;
    feedback->detail = detail;
    active_goal_->publish_feedback(feedback);
  }

  void finish_active_action(
    ActionExecutionState state, const std::string & message)
  {
    if (!active_goal_) {
      return;
    }
    executor_->stop();
    // reset 前先保存名称，确保最终 ACK 仍能关联到刚完成的 goal。
    const std::string action_name = action_runtime_ ?
      action_runtime_->action_name() : "unknown";
    auto result = std::make_shared<ExecuteRobotCommand::Result>();
    result->success = state == ActionExecutionState::kSucceeded;
    result->message = message;
    if (state == ActionExecutionState::kSucceeded) {
      result->status = ExecuteRobotCommand::Result::STATUS_SUCCEEDED;
      active_goal_->succeed(result);
    } else if (state == ActionExecutionState::kCanceled) {
      result->status = ExecuteRobotCommand::Result::STATUS_CANCELED;
      if (active_goal_->is_canceling()) {
        active_goal_->canceled(result);
      } else {
        active_goal_->abort(result);
      }
    } else if (state == ActionExecutionState::kTimedOut) {
      result->status = ExecuteRobotCommand::Result::STATUS_TIMED_OUT;
      active_goal_->abort(result);
    } else {
      result->status = ExecuteRobotCommand::Result::STATUS_BLOCKED;
      active_goal_->abort(result);
    }
    publish_action_ack(action_name, message);
    active_goal_.reset();
    action_active_ = false;
    if (action_runtime_) {
      action_runtime_->reset();
    }
  }

  bool update_active_action(const ControllerOutput & output, double now)
  {
    if (!active_goal_ || !action_runtime_ || !action_runtime_->active()) {
      return false;
    }
    ActiveActionInput input;
    input.now_s = now;
    input.cancel_requested = active_goal_->is_canceling();
    input.safety_stopped = output.safety_stopped;
    input.safety_reason = output.reason;
    if (action_runtime_->uses_external_result()) {
      // Nav2 是外部 Action Server，完成/失败以其 result 为准；本地计时只负责
      // 进度下限和最终超时，避免 duration_s 到点便错误宣告到达目标。
      input.external_update = executor_->external_action_update();
      input.external_detail = executor_->external_action_detail();
    }
    const auto decision = action_runtime_->update(input);
    if (decision.tree_result) {
      publish_bt_status(*decision.tree_result);
    }
    if (!decision.terminal()) {
      publish_action_feedback(
        ExecuteRobotCommand::Feedback::PHASE_EXECUTING,
        static_cast<float>(decision.progress), decision.detail);
      return false;
    }
    finish_active_action(decision.state, decision.detail);
    return true;
  }

  void on_mode_request(const std_msgs::msg::String::SharedPtr message)
  {
    if (!is_active()) {
      return;
    }
    set_mode(message->data);
  }

  void on_emergency_stop(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!is_active()) {
      return;
    }
    executor_->stop();
    publish_mode();
    publish_action_ack("stop", "accepted", "emergency_stop");
    RCLCPP_WARN(get_logger(), "emergency stop received; switched to manual");
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    if (!is_active()) {
      return;
    }
    executor_->update_scan(
      message->ranges, message->angle_min, message->angle_increment,
      message->range_min, message->range_max, now_seconds());
  }

  bool set_mode(const std::string & mode)
  {
    embodied_agent_interfaces::msg::RobotCommand command;
    command.action_type = command.SET_MODE;
    command.mode = mode;
    if (!executor_->execute(command, now_seconds())) {
      RCLCPP_WARN(get_logger(), "unsupported control mode: %s", mode.c_str());
      return false;
    }
    publish_mode();
    RCLCPP_INFO(get_logger(), "control mode changed to %s", mode.c_str());
    return true;
  }

  void publish_action_ack(
    const std::string & action,
    const std::string & status,
    const std::string & detail = "")
  {
    embodied_agent_interfaces::msg::RobotActionAck message;
    message.stamp = now();
    message.action = action;
    message.backend = executor_ ? executor_->backend_name() : "unconfigured";
    message.sequence = ++action_sequence_;
    message.status = action_ack_status(status);
    message.detail = detail;
    action_ack_pub_->publish(message);
  }

  static uint8_t action_ack_status(const std::string & status)
  {
    using Ack = embodied_agent_interfaces::msg::RobotActionAck;
    if (status == "accepted") {return Ack::STATUS_ACCEPTED;}
    if (status == "rejected") {return Ack::STATUS_REJECTED;}
    if (status == "succeeded") {return Ack::STATUS_SUCCEEDED;}
    if (status == "canceled") {return Ack::STATUS_CANCELED;}
    if (status == "timed_out") {return Ack::STATUS_TIMED_OUT;}
    if (status == "blocked") {return Ack::STATUS_BLOCKED;}
    return Ack::STATUS_UNKNOWN;
  }

  static const char * tree_outcome_name(CommandTreeOutcome outcome)
  {
    switch (outcome) {
      case CommandTreeOutcome::kRunning: return "running";
      case CommandTreeOutcome::kSucceeded: return "succeeded";
      case CommandTreeOutcome::kRejected: return "rejected";
      case CommandTreeOutcome::kCanceled: return "canceled";
      case CommandTreeOutcome::kTimedOut: return "timed_out";
      case CommandTreeOutcome::kBlocked: return "blocked";
      case CommandTreeOutcome::kFailed: return "failed";
    }
    return "failed";
  }

  static uint8_t tree_outcome_value(CommandTreeOutcome outcome)
  {
    using Status = embodied_agent_interfaces::msg::BehaviorTreeStatus;
    switch (outcome) {
      case CommandTreeOutcome::kRunning: return Status::OUTCOME_RUNNING;
      case CommandTreeOutcome::kSucceeded: return Status::OUTCOME_SUCCEEDED;
      case CommandTreeOutcome::kRejected: return Status::OUTCOME_REJECTED;
      case CommandTreeOutcome::kCanceled: return Status::OUTCOME_CANCELED;
      case CommandTreeOutcome::kTimedOut: return Status::OUTCOME_TIMED_OUT;
      case CommandTreeOutcome::kBlocked: return Status::OUTCOME_BLOCKED;
      case CommandTreeOutcome::kFailed: return Status::OUTCOME_FAILED;
    }
    return Status::OUTCOME_UNKNOWN;
  }

  void publish_bt_status(const CommandTreeResult & result)
  {
    if (!bt_status_pub_ || !bt_status_pub_->is_activated()) {
      return;
    }
    const std::string command_id = active_goal_ ?
      active_goal_->get_goal()->command.command_id : "";
    const std::string signature = command_id + ":" + result.stage + ":" +
      tree_outcome_name(result.outcome) + ":" + result.detail;
    if (signature == last_bt_status_) {
      return;
    }
    last_bt_status_ = signature;
    embodied_agent_interfaces::msg::BehaviorTreeStatus message;
    message.stamp = now();
    message.command_id = command_id;
    message.stage = result.stage;
    message.outcome = tree_outcome_value(result.outcome);
    message.detail = result.detail;
    bt_status_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(), "BT %s -> %s (%s)", result.stage.c_str(),
      tree_outcome_name(result.outcome), result.detail.c_str());
  }

  void publish_mode()
  {
    std_msgs::msg::String message;
    message.data = executor_->mode_name();
    mode_pub_->publish(message);
  }

  void publish_health(const std::uint8_t state, const std::string & detail)
  {
    if (!health_pub_ || !health_pub_->is_activated()) {
      return;
    }
    embodied_agent_interfaces::msg::ComponentHealth message;
    message.stamp = now();
    message.component = "simulation_control";
    message.state = state;
    message.detail = detail;
    health_pub_->publish(message);
  }

  void publish_zero_velocity()
  {
    if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
      return;
    }
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_diagnostics()
  {
    if (!diagnostics_pub_ || !diagnostics_pub_->is_activated() || !executor_) {
      return;
    }
    ControllerOutput output;
    {
      std::lock_guard<std::mutex> lock(diagnostics_mutex_);
      output = diagnostic_output_;
    }
    // 回调组可以并行运行；传给纯函数的内容全部来自锁保护快照或只读配置。
    const auto status = make_executor_diagnostic({
      get_fully_qualified_name(), get_current_state().label(), executor_plugin_,
      executor_backend_, SimulationController::mode_name(output.mode), output.reason,
      action_active_, output.sensor_stale, output.safety_stopped});
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(status);
    diagnostics_pub_->publish(message);
    // readiness 使用周期心跳判断进程是否仍存活，不能只依赖启动时的一次 latched 状态。
    publish_health(
      embodied_agent_interfaces::msg::ComponentHealth::STATE_READY,
      "executor_ready:" + executor_backend_);
  }

  void reset_interfaces()
  {
    if (timer_) {
      timer_->cancel();
    }
    timer_.reset();
    if (diagnostics_timer_) {
      diagnostics_timer_->cancel();
    }
    diagnostics_timer_.reset();
    diagnostics_callback_group_.reset();
    action_server_.reset();
    scan_sub_.reset();
    emergency_sub_.reset();
    mode_sub_.reset();
    diagnostics_pub_.reset();
    health_pub_.reset();
    bt_status_pub_.reset();
    action_ack_pub_.reset();
    state_pub_.reset();
    mode_pub_.reset();
    cmd_vel_pub_.reset();
    active_goal_.reset();
    action_active_ = false;
    if (action_runtime_) {
      action_runtime_->reset();
    }
    action_runtime_.reset();
    action_sequence_ = 0;
    executor_backend_.clear();
    behavior_tree_.reset();
    last_bt_status_.clear();
  }

  void control_tick()
  {
    const double now = now_seconds();
    const auto output = executor_->step(now);
    {
      std::lock_guard<std::mutex> lock(diagnostics_mutex_);
      diagnostic_output_ = output;
    }
    const bool action_stopped = update_active_action(output, now);
    if (executor_->publishes_cmd_vel()) {
      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = action_stopped ? 0.0 : output.velocity.linear_x;
      velocity.angular.z = action_stopped ? 0.0 : output.velocity.angular_z;
      cmd_vel_pub_->publish(velocity);
    }

    embodied_agent_interfaces::msg::SimulationState state_message;
    state_message.stamp = this->now();
    state_message.mode = SimulationController::mode_name(output.mode);
    state_message.sensor_stale = output.sensor_stale;
    state_message.safety_stopped = output.safety_stopped;
    state_message.front_distance_valid = std::isfinite(output.front_distance);
    state_message.front_distance = state_message.front_distance_valid ?
      static_cast<float>(output.front_distance) : 0.0F;
    state_message.right_distance_valid = std::isfinite(output.right_distance);
    state_message.right_distance = state_message.right_distance_valid ?
      static_cast<float>(output.right_distance) : 0.0F;
    state_message.reason = output.reason;
    state_message.linear_x = static_cast<float>(output.velocity.linear_x);
    state_message.angular_z = static_cast<float>(output.velocity.angular_z);
    state_pub_->publish(state_message);
  }

  pluginlib::ClassLoader<RobotExecutor> executor_loader_;
  std::shared_ptr<RobotExecutor> executor_;
  std::string executor_plugin_;
  std::string executor_backend_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::SimulationState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::RobotActionAck>::SharedPtr
    action_ack_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::BehaviorTreeStatus>::SharedPtr
    bt_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::ComponentHealth>::SharedPtr health_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;
  std::uint64_t action_sequence_{0};
  double action_timeout_s_{12.0};
  rclcpp_action::Server<ExecuteRobotCommand>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  bool use_behavior_tree_{true};
  std::unique_ptr<CommandBehaviorTree> behavior_tree_;
  std::unique_ptr<ActiveActionRuntime> action_runtime_;
  std::string last_bt_status_;
  std::atomic_bool action_active_{false};
  std::mutex diagnostics_mutex_;
  ControllerOutput diagnostic_output_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_simulation_control_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<SimulationControlNode>(options);
}

}  // namespace embodied_simulation

RCLCPP_COMPONENTS_REGISTER_NODE(embodied_simulation::SimulationControlNode)
