#include "embodied_simulation/simulation_ros_io.hpp"

#include <cmath>
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_simulation
{

SimulationRosIo::SimulationRosIo(rclcpp_lifecycle::LifecycleNode & node)
: node_(node)
{
}

void SimulationRosIo::configure()
{
  cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", embodied_agent_middleware::command_qos(10));
  mode_pub_ = node_.create_publisher<std_msgs::msg::String>(
    "robot/control_mode", embodied_agent_middleware::state_qos());
  state_pub_ = node_.create_publisher<embodied_agent_interfaces::msg::SimulationState>(
    "robot/simulation_state", embodied_agent_middleware::state_qos());
  action_ack_pub_ =
    node_.create_publisher<embodied_agent_interfaces::msg::RobotActionAck>(
    "robot/action_ack", embodied_agent_middleware::event_qos());
  bt_status_pub_ =
    node_.create_publisher<embodied_agent_interfaces::msg::BehaviorTreeStatus>(
    "robot/bt_status", embodied_agent_middleware::event_qos());
  diagnostics_pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", embodied_agent_middleware::diagnostics_qos());
  health_pub_ = node_.create_publisher<embodied_agent_interfaces::msg::ComponentHealth>(
    "system/component_health", embodied_agent_middleware::state_qos());
}

void SimulationRosIo::activate(
  const std::string & mode, const std::string & backend)
{
  cmd_vel_pub_->on_activate();
  mode_pub_->on_activate();
  state_pub_->on_activate();
  action_ack_pub_->on_activate();
  bt_status_pub_->on_activate();
  diagnostics_pub_->on_activate();
  health_pub_->on_activate();
  publish_mode(mode);
  publish_health(
    embodied_agent_interfaces::msg::ComponentHealth::STATE_READY,
    "executor_ready:" + backend);
}

void SimulationRosIo::deactivate(const std::string & detail)
{
  // STOPPED 必须在 managed publisher 停用前覆盖 transient-local READY 缓存。
  publish_health(
    embodied_agent_interfaces::msg::ComponentHealth::STATE_STOPPED, detail);
  health_pub_->on_deactivate();
  diagnostics_pub_->on_deactivate();
  bt_status_pub_->on_deactivate();
  action_ack_pub_->on_deactivate();
  state_pub_->on_deactivate();
  mode_pub_->on_deactivate();
  cmd_vel_pub_->on_deactivate();
}

void SimulationRosIo::reset()
{
  diagnostics_pub_.reset();
  health_pub_.reset();
  bt_status_pub_.reset();
  action_ack_pub_.reset();
  state_pub_.reset();
  mode_pub_.reset();
  cmd_vel_pub_.reset();
  action_sequence_ = 0;
  last_bt_status_.clear();
}

void SimulationRosIo::publish_velocity(double linear_x, double angular_z)
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }
  geometry_msgs::msg::Twist message;
  message.linear.x = linear_x;
  message.angular.z = angular_z;
  cmd_vel_pub_->publish(message);
}

void SimulationRosIo::publish_zero_velocity()
{
  publish_velocity(0.0, 0.0);
}

void SimulationRosIo::publish_state(const ControllerOutput & output)
{
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }
  embodied_agent_interfaces::msg::SimulationState message;
  message.stamp = node_.now();
  message.mode = SimulationController::mode_name(output.mode);
  message.sensor_stale = output.sensor_stale;
  message.safety_stopped = output.safety_stopped;
  message.front_distance_valid = std::isfinite(output.front_distance);
  message.front_distance = message.front_distance_valid ?
    static_cast<float>(output.front_distance) : 0.0F;
  message.right_distance_valid = std::isfinite(output.right_distance);
  message.right_distance = message.right_distance_valid ?
    static_cast<float>(output.right_distance) : 0.0F;
  message.reason = output.reason;
  message.linear_x = static_cast<float>(output.velocity.linear_x);
  message.angular_z = static_cast<float>(output.velocity.angular_z);
  state_pub_->publish(message);
}

void SimulationRosIo::publish_mode(const std::string & mode)
{
  if (!mode_pub_ || !mode_pub_->is_activated()) {
    return;
  }
  std_msgs::msg::String message;
  message.data = mode;
  mode_pub_->publish(message);
}

void SimulationRosIo::publish_action_ack(
  const std::string & action, const std::string & status,
  const std::string & backend, const std::string & detail)
{
  if (!action_ack_pub_ || !action_ack_pub_->is_activated()) {
    return;
  }
  embodied_agent_interfaces::msg::RobotActionAck message;
  message.stamp = node_.now();
  message.action = action;
  message.backend = backend;
  message.sequence = ++action_sequence_;
  message.status = action_ack_status(status);
  message.detail = detail;
  action_ack_pub_->publish(message);
}

void SimulationRosIo::publish_bt_status(
  const CommandTreeResult & result, const std::string & command_id)
{
  if (!bt_status_pub_ || !bt_status_pub_->is_activated()) {
    return;
  }
  const std::string signature = command_id + ":" + result.stage + ":" +
    tree_outcome_name(result.outcome) + ":" + result.detail;
  if (signature == last_bt_status_) {
    return;
  }
  last_bt_status_ = signature;
  embodied_agent_interfaces::msg::BehaviorTreeStatus message;
  message.stamp = node_.now();
  message.command_id = command_id;
  message.stage = result.stage;
  message.outcome = tree_outcome_value(result.outcome);
  message.detail = result.detail;
  bt_status_pub_->publish(message);
  RCLCPP_INFO(
    node_.get_logger(), "BT %s -> %s (%s)", result.stage.c_str(),
    tree_outcome_name(result.outcome), result.detail.c_str());
}

void SimulationRosIo::publish_health(
  const std::uint8_t state, const std::string & detail)
{
  if (!health_pub_ || !health_pub_->is_activated()) {
    return;
  }
  embodied_agent_interfaces::msg::ComponentHealth message;
  message.stamp = node_.now();
  message.component = "simulation_control";
  message.state = state;
  message.detail = detail;
  health_pub_->publish(message);
}

void SimulationRosIo::publish_diagnostics(
  const ExecutorDiagnosticSnapshot & snapshot)
{
  if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = node_.now();
  message.status.push_back(make_executor_diagnostic(snapshot));
  diagnostics_pub_->publish(message);
}

std::uint8_t SimulationRosIo::action_ack_status(const std::string & status)
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

const char * SimulationRosIo::tree_outcome_name(CommandTreeOutcome outcome)
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

std::uint8_t SimulationRosIo::tree_outcome_value(CommandTreeOutcome outcome)
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

}  // namespace embodied_simulation
