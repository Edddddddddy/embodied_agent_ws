#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <embodied_agent_interfaces/action/execute_robot_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "embodied_simulation/action_execution.hpp"
#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

class SimulationControlNode : public rclcpp::Node
{
public:
  using ExecuteRobotCommand =
    embodied_agent_interfaces::action::ExecuteRobotCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteRobotCommand>;

  SimulationControlNode()
  : Node("simulation_control"), controller_(load_config())
  {
    using std::placeholders::_1;
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/robot/control_mode", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/robot/simulation_state", 10);
    action_ack_pub_ = create_publisher<std_msgs::msg::String>("/robot/action_ack", 10);
    const bool legacy_command_enabled =
      declare_parameter("legacy_command_enabled", true);
    action_timeout_s_ = declare_parameter("action_timeout_s", 12.0);
    if (legacy_command_enabled) {
      action_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot/action_command", 10,
        std::bind(&SimulationControlNode::on_action, this, _1));
    }
    action_server_ = rclcpp_action::create_server<ExecuteRobotCommand>(
      this,
      "/robot/execute_command",
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
      "/robot/control_mode_request", 10,
      std::bind(&SimulationControlNode::on_mode_request, this, _1));
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/robot/emergency_stop", 10,
      std::bind(&SimulationControlNode::on_emergency_stop, this, _1));

    auto scan_qos = rclcpp::SensorDataQoS().keep_last(5);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos,
      std::bind(&SimulationControlNode::on_scan, this, _1));

    const auto period = std::chrono::duration<double>(
      1.0 / get_parameter("control_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimulationControlNode::control_tick, this));
    publish_mode();
    RCLCPP_INFO(get_logger(), "simulation control ready; mode=manual");
  }

private:
  ControllerConfig load_config()
  {
    ControllerConfig config;
    config.max_linear_speed = declare_parameter("max_linear_speed", 0.22);
    config.max_angular_speed = declare_parameter("max_angular_speed", 1.0);
    config.linear_acceleration = declare_parameter("linear_acceleration", 0.5);
    config.angular_acceleration = declare_parameter("angular_acceleration", 2.0);
    config.emergency_distance = declare_parameter("emergency_distance", 0.22);
    config.obstacle_distance = declare_parameter("obstacle_distance", 0.50);
    config.wall_target_distance = declare_parameter("wall_target_distance", 0.40);
    config.autonomous_linear_speed = declare_parameter("autonomous_linear_speed", 0.14);
    config.obstacle_turn_speed = declare_parameter("obstacle_turn_speed", 0.65);
    config.scan_timeout = declare_parameter("scan_timeout", 0.50);
    config.wall_kp = declare_parameter("wall_kp", 1.8);
    config.wall_ki = declare_parameter("wall_ki", 0.0);
    config.wall_kd = declare_parameter("wall_kd", 0.15);
    declare_parameter("control_rate_hz", 20.0);
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
    if (command.action_type == Command::MOVE) {
      return std::isfinite(command.linear_x) &&
             std::isfinite(command.duration_s) &&
             command.duration_s >= 0.0 && command.duration_s <= 10.0;
    }
    if (command.action_type == Command::TURN) {
      return std::isfinite(command.angular_z) &&
             std::isfinite(command.duration_s) &&
             command.duration_s >= 0.0 && command.duration_s <= 10.0;
    }
    return false;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteRobotCommand::Goal> goal)
  {
    if (!supported_action_goal(goal->command)) {
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
      finish_active_action(
        ActionExecutionState::kCanceled, "preempted_by_new_goal");
    }

    const auto & command = goal_handle->get_goal()->command;
    using Command = embodied_agent_interfaces::msg::RobotCommand;
    if (command.action_type == Command::STOP) {
      controller_.stop();
      finish_immediate_action(goal_handle, true, "stopped");
      publish_action_ack("stop", "accepted");
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

    const double now = now_seconds();
    if (command.action_type == Command::MOVE) {
      controller_.set_manual_command(
        command.linear_x, 0.0, command.duration_s, now);
      active_action_name_ = "move";
    } else {
      controller_.set_manual_command(
        0.0, command.angular_z, command.duration_s, now);
      active_action_name_ = "turn";
    }
    active_goal_ = goal_handle;
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
    controller_.stop();
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
    try {
      const auto command = nlohmann::json::parse(message->data);
      const std::string name = command.at("name").get<std::string>();
      const auto & arguments = command.at("arguments");
      if (name == "move") {
        controller_.set_manual_command(
          arguments.at("linear_x").get<double>(), 0.0,
          arguments.at("duration_s").get<double>(), now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "turn") {
        controller_.set_manual_command(
          0.0, arguments.at("angular_z").get<double>(),
          arguments.at("duration_s").get<double>(), now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "stop") {
        controller_.stop();
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "set_mode") {
        const bool accepted = set_mode(arguments.at("mode").get<std::string>());
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
    set_mode(message->data);
  }

  void on_emergency_stop(const std_msgs::msg::Empty::SharedPtr)
  {
    controller_.stop();
    publish_mode();
    publish_action_ack("stop", "accepted", "emergency_stop");
    RCLCPP_WARN(get_logger(), "emergency stop received; switched to manual");
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    controller_.update_scan(
      message->ranges, message->angle_min, message->angle_increment,
      message->range_min, message->range_max, now_seconds());
  }

  bool set_mode(const std::string & mode)
  {
    if (!controller_.set_mode(mode)) {
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
      {"backend", "simulation"},
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

  void publish_mode()
  {
    std_msgs::msg::String message;
    message.data = SimulationController::mode_name(controller_.mode());
    mode_pub_->publish(message);
  }

  void control_tick()
  {
    const double now = now_seconds();
    const auto output = controller_.step(now);
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

  SimulationController controller_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_ack_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t action_sequence_{0};
  double action_timeout_s_{12.0};
  rclcpp_action::Server<ExecuteRobotCommand>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::optional<ActionExecution> action_execution_;
  std::string active_action_name_;
};

}  // namespace embodied_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_simulation::SimulationControlNode>());
  rclcpp::shutdown();
  return 0;
}
