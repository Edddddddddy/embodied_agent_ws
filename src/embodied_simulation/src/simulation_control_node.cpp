#include <atomic>
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "embodied_simulation/action_execution.hpp"
#include "embodied_simulation/command_behavior_tree.hpp"
#include "embodied_simulation/executor_diagnostics.hpp"
#include "embodied_simulation/node_configuration.hpp"
#include "embodied_simulation/robot_executor.hpp"
#include "embodied_simulation/simulation_controller.hpp"
#include "embodied_simulation/simulation_control_factory.hpp"

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
      "cmd_vel", rclcpp::QoS(10).reliable());
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/control_mode", rclcpp::QoS(10).reliable());
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/simulation_state", rclcpp::QoS(10).reliable());
    action_ack_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/action_ack", rclcpp::QoS(10).reliable());
    bt_status_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/bt_status", rclcpp::QoS(10).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", rclcpp::QoS(10).reliable());
    const bool legacy_command_enabled =
      bool_parameter("legacy_command_enabled", true);
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
    if (legacy_command_enabled) {
      action_sub_ = create_subscription<std_msgs::msg::String>(
        "robot/action_command", rclcpp::QoS(10).reliable(),
        std::bind(&SimulationControlNode::on_action, this, _1));
    }
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
      "robot/control_mode_request", rclcpp::QoS(10).reliable(),
      std::bind(&SimulationControlNode::on_mode_request, this, _1));
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
      "robot/emergency_stop", rclcpp::QoS(10).reliable(),
      std::bind(&SimulationControlNode::on_emergency_stop, this, _1));

    auto scan_qos = rclcpp::SensorDataQoS().keep_last(5);
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
    timer_->reset();
    diagnostics_timer_->reset();
    publish_mode();
    RCLCPP_INFO(get_logger(), "simulation control activated; mode=manual");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (active_goal_) {
      if (behavior_tree_) {
        publish_bt_status(behavior_tree_->cancel("deactivated"));
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
      if (behavior_tree_) {
        publish_bt_status(behavior_tree_->cancel("preempted_by_new_goal"));
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
        active_goal_.reset();
        action_active_ = false;
        return;
      }
    }

    const double now = now_seconds();
    if (command.action_type == Command::MOVE) {
      if (!executor_->execute(command, now)) {
        finish_immediate_action(goal_handle, false, "executor_rejected");
        active_goal_.reset();
        return;
      }
      active_action_name_ = "move";
    } else if (command.action_type == Command::TURN) {
      if (!executor_->execute(command, now)) {
        finish_immediate_action(goal_handle, false, "executor_rejected");
        active_goal_.reset();
        return;
      }
      active_action_name_ = "turn";
    } else if (command.action_type == Command::NAVIGATE_TO) {
      if (!executor_->execute(command, now)) {
        finish_immediate_action(goal_handle, false, "executor_rejected");
        active_goal_.reset();
        return;
      }
      active_action_name_ = "navigate_to";
    } else if (command.action_type == Command::FOLLOW_WAYPOINTS) {
      if (!executor_->execute(command, now)) {
        finish_immediate_action(goal_handle, false, "executor_rejected");
        active_goal_.reset();
        return;
      }
      active_action_name_ = "follow_waypoints";
    } else {
      finish_immediate_action(goal_handle, false, "invalid_command");
      active_goal_.reset();
      return;
    }
    active_goal_ = goal_handle;
    action_active_ = true;
    action_execution_.emplace(
      command.duration_s, action_timeout_s_, now);
    publish_action_feedback(
      ExecuteRobotCommand::Feedback::PHASE_ACCEPTED, 0.0F, "accepted");
    publish_action_ack(active_action_name_, "accepted");
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
    publish_action_ack(active_action_name_, message);
    active_goal_.reset();
    action_active_ = false;
    action_execution_.reset();
    active_action_name_.clear();
  }

  bool update_active_action(const ControllerOutput & output, double now)
  {
    if (!active_goal_ || !action_execution_) {
      return false;
    }
    const auto update = action_execution_->update(
      now, active_goal_->is_canceling(), output.safety_stopped);
    if (behavior_tree_) {
      const std::string detail =
        update.state == ActionExecutionState::kSucceeded ? "succeeded" :
        update.state == ActionExecutionState::kCanceled ? "canceled" :
        update.state == ActionExecutionState::kTimedOut ? "timed_out" :
        output.reason;
      const auto tree_result = behavior_tree_->tick(
        output.safety_stopped, update.state, detail);
      publish_bt_status(tree_result);
      if (tree_result.outcome == CommandTreeOutcome::kRunning) {
        publish_action_feedback(
          ExecuteRobotCommand::Feedback::PHASE_EXECUTING,
          static_cast<float>(update.progress), tree_result.stage + ":" + detail);
        return false;
      }
      if (tree_result.outcome == CommandTreeOutcome::kSucceeded) {
        finish_active_action(ActionExecutionState::kSucceeded, tree_result.detail);
      } else if (tree_result.outcome == CommandTreeOutcome::kCanceled) {
        finish_active_action(ActionExecutionState::kCanceled, tree_result.detail);
      } else if (tree_result.outcome == CommandTreeOutcome::kTimedOut) {
        finish_active_action(ActionExecutionState::kTimedOut, tree_result.detail);
      } else {
        finish_active_action(ActionExecutionState::kBlocked, tree_result.detail);
      }
      return true;
    }
    if (update.state == ActionExecutionState::kRunning) {
      publish_action_feedback(
        ExecuteRobotCommand::Feedback::PHASE_EXECUTING,
        static_cast<float>(update.progress), output.reason);
      return false;
    }
    switch (update.state) {
      case ActionExecutionState::kSucceeded:
        finish_active_action(update.state, "succeeded");
        break;
      case ActionExecutionState::kCanceled:
        finish_active_action(update.state, "canceled");
        break;
      case ActionExecutionState::kBlocked:
        finish_active_action(update.state, output.reason);
        break;
      case ActionExecutionState::kTimedOut:
        finish_active_action(update.state, "timed_out");
        break;
      case ActionExecutionState::kRunning:
        break;
    }
    return true;
  }

  void on_action(const std_msgs::msg::String::SharedPtr message)
  {
    if (!is_active()) {
      return;
    }
    try {
      const auto command = nlohmann::json::parse(message->data);
      const std::string name = command.at("name").get<std::string>();
      const auto & arguments = command.at("arguments");
      embodied_agent_interfaces::msg::RobotCommand typed_command;
      if (name == "move") {
        typed_command.action_type = typed_command.MOVE;
        typed_command.linear_x = arguments.at("linear_x").get<double>();
        typed_command.angular_z = arguments.value("angular_z", 0.0);
        typed_command.duration_s = arguments.at("duration_s").get<double>();
        executor_->execute(typed_command, now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "turn") {
        typed_command.action_type = typed_command.TURN;
        typed_command.angular_z = arguments.at("angular_z").get<double>();
        typed_command.duration_s = arguments.at("duration_s").get<double>();
        executor_->execute(typed_command, now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "stop") {
        executor_->stop();
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "navigate_to") {
        typed_command.action_type = typed_command.NAVIGATE_TO;
        typed_command.target = arguments.at("target").get<std::string>();
        typed_command.duration_s = 3.0;
        executor_->execute(typed_command, now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "follow_waypoints") {
        typed_command.action_type = typed_command.FOLLOW_WAYPOINTS;
        for (const auto & waypoint : arguments.at("waypoints")) {
          typed_command.waypoints.push_back(waypoint.get<std::string>());
        }
        typed_command.number_of_loops = arguments.value("number_of_loops", 1);
        typed_command.duration_s = std::min(
          10.0,
          std::max(
            2.0,
            static_cast<double>(
              typed_command.waypoints.size() *
              std::max<std::uint32_t>(1U, typed_command.number_of_loops)) * 2.0));
        executor_->execute(typed_command, now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "cancel_navigation") {
        executor_->stop();
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "set_mode") {
        const bool accepted = set_mode(arguments.at("mode").get<std::string>());
        publish_action_ack(name, accepted ? "accepted" : "rejected");
      } else if (name == "wave") {
        typed_command.action_type = typed_command.WAVE;
        typed_command.count = arguments.at("count").get<int>();
        const bool accepted = executor_->execute(typed_command, now_seconds());
        publish_action_ack(name, accepted ? "accepted" : "rejected");
      } else if (name == "set_led") {
        typed_command.action_type = typed_command.SET_LED;
        typed_command.color = arguments.at("color").get<std::string>();
        const bool accepted = executor_->execute(typed_command, now_seconds());
        publish_action_ack(name, accepted ? "accepted" : "rejected");
      } else {
        publish_action_ack(name, "ignored");
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "ignored malformed trusted action: %s", error.what());
      publish_action_ack("unknown", "rejected", error.what());
    }
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
    nlohmann::json payload{
      {"action", action},
      {"backend", executor_ ? executor_->backend_name() : "unconfigured"},
      {"sequence", ++action_sequence_},
      {"status", status},
    };
    if (!detail.empty()) {
      payload["detail"] = detail;
    }
    std_msgs::msg::String message;
    message.data = payload.dump();
    action_ack_pub_->publish(message);
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
    std_msgs::msg::String message;
    message.data = nlohmann::json{
      {"command_id", command_id},
      {"stage", result.stage},
      {"outcome", tree_outcome_name(result.outcome)},
      {"detail", result.detail},
    }.dump();
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
    action_sub_.reset();
    diagnostics_pub_.reset();
    bt_status_pub_.reset();
    action_ack_pub_.reset();
    state_pub_.reset();
    mode_pub_.reset();
    cmd_vel_pub_.reset();
    active_goal_.reset();
    action_active_ = false;
    action_execution_.reset();
    active_action_name_.clear();
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
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = action_stopped ? 0.0 : output.velocity.linear_x;
    velocity.angular.z = action_stopped ? 0.0 : output.velocity.angular_z;
    cmd_vel_pub_->publish(velocity);

    nlohmann::json state{
      {"mode", SimulationController::mode_name(output.mode)},
      {"sensor_stale", output.sensor_stale},
      {"safety_stopped", output.safety_stopped},
      {"front_distance", std::isfinite(output.front_distance) ?
        nlohmann::json(output.front_distance) : nlohmann::json(nullptr)},
      {"right_distance", std::isfinite(output.right_distance) ?
        nlohmann::json(output.right_distance) : nlohmann::json(nullptr)},
      {"reason", output.reason},
      {"linear_x", output.velocity.linear_x},
      {"angular_z", output.velocity.angular_z},
    };
    std_msgs::msg::String state_message;
    state_message.data = state.dump();
    state_pub_->publish(state_message);
  }

  pluginlib::ClassLoader<RobotExecutor> executor_loader_;
  std::shared_ptr<RobotExecutor> executor_;
  std::string executor_plugin_;
  std::string executor_backend_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    action_ack_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    bt_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
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
  std::optional<ActionExecution> action_execution_;
  std::string active_action_name_;
  bool use_behavior_tree_{true};
  std::unique_ptr<CommandBehaviorTree> behavior_tree_;
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
