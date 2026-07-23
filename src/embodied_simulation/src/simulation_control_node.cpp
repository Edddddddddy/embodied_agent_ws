#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <embodied_agent_interfaces/action/execute_robot_command.hpp>
#include <embodied_agent_interfaces/msg/component_health.hpp>
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
#include "embodied_simulation/robot_command_policy.hpp"
#include "embodied_simulation/simulation_controller.hpp"
#include "embodied_simulation/simulation_control_factory.hpp"
#include "embodied_simulation/simulation_ros_io.hpp"
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
    ros_io_ = std::make_unique<SimulationRosIo>(*this);
    ros_io_->configure();
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
    ros_io_->activate(executor_->mode_name(), executor_backend_);
    timer_->reset();
    diagnostics_timer_->reset();
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
      executor_->request_stop(StopClock::now());
    }
    ros_io_->publish_zero_velocity();
    if (timer_) {
      timer_->cancel();
    }
    if (diagnostics_timer_) {
      diagnostics_timer_->cancel();
    }
    ros_io_->deactivate("lifecycle_deactivated");
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
      executor_->request_stop(StopClock::now());
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
    config.stop_timeout_s = double_parameter("stop_timeout_s", 3.0);
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

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteRobotCommand::Goal> goal)
  {
    if (!is_active()) {
      RCLCPP_WARN(get_logger(), "rejected typed action goal while inactive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!is_executable_robot_command(goal->command) && !behavior_tree_) {
      RCLCPP_WARN(get_logger(), "rejected unsupported typed action goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    using Command = embodied_agent_interfaces::msg::RobotCommand;
    const bool is_stop =
      goal->command.action_type == Command::STOP ||
      goal->command.action_type == Command::CANCEL_NAVIGATION;
    if (!is_stop && executor_ && !executor_->is_quiesced()) {
      // 上一个 Nav2 goal 未收到 terminal 前，普通命令不能重新获得速度控制权；
      // STOP/CANCEL 仍可进入，以便调用方等待真正的 quiescence。
      RCLCPP_WARN(get_logger(), "rejected goal while executor is not quiesced");
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
      start_stop_action(goal_handle, "stop");
      return;
    }
    if (command.action_type == Command::CANCEL_NAVIGATION) {
      start_stop_action(goal_handle, "cancel_navigation");
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

  void start_stop_action(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & action_name)
  {
    const auto requested_at = StopClock::now();
    executor_->request_stop(requested_at);
    if (executor_->publishes_cmd_vel()) {
      // 本地速度 adapter 可立即归零；Nav2 adapter 没有 /cmd_vel 写权限，
      // 必须等待其 Action terminal，而不是额外抢写速度话题。
      ros_io_->publish_zero_velocity();
    }
    const auto stop = executor_->poll_stop(requested_at);
    if (stop.succeeded()) {
      finish_immediate_action(goal_handle, true, stop.detail);
      publish_action_ack(action_name, "accepted", stop.detail);
      return;
    }
    if (stop.terminal()) {
      finish_immediate_action(goal_handle, false, stop.detail);
      publish_action_ack(action_name, "rejected", stop.detail);
      return;
    }

    active_goal_ = goal_handle;
    action_active_ = true;
    if (behavior_tree_) {
      behavior_tree_->start(goal_handle->get_goal()->command);
      const auto initial = behavior_tree_->tick(
        false, ActionExecutionState::kRunning, stop.detail);
      publish_bt_status(initial);
    }
    // STOP 自身由 executor 的 quiescence 状态驱动；本地 hard timeout 只作为
    // 第二道保险，不能用固定 sleep 代替底层 NavigateToPose result。
    action_runtime_->start(
      action_name, action_timeout_s_ + 1.0, action_timeout_s_,
      now_seconds(), true);
    publish_action_feedback(
      ExecuteRobotCommand::Feedback::PHASE_STOPPING, 0.0F, stop.detail);
    // ACK 仅表示 STOP 请求已被本节点接受；是否真正停止仍以 typed Action
    // terminal result 为准，等待期间通过 PHASE_STOPPING 持续反馈。
    publish_action_ack(action_name, "accepted", "stopping;" + stop.detail);
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
    executor_->request_stop(StopClock::now());
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
      const auto action_name = action_runtime_->action_name();
      if (action_name == "stop" || action_name == "cancel_navigation") {
        const auto stop = executor_->poll_stop(StopClock::now());
        input.external_update = stop_as_action_update(stop);
        input.external_detail = stop.detail;
      } else {
        // Nav2 是外部 Action Server，完成/失败以其 result 为准；本地计时只负责
        // 进度下限和最终超时，避免 duration_s 到点便错误宣告到达目标。
        input.external_update = executor_->external_action_update();
        input.external_detail = executor_->external_action_detail();
      }
    }
    const auto decision = action_runtime_->update(input);
    if (decision.tree_result) {
      publish_bt_status(*decision.tree_result);
    }
    if (!decision.terminal()) {
      const bool stopping =
        action_runtime_->action_name() == "stop" ||
        action_runtime_->action_name() == "cancel_navigation";
      publish_action_feedback(
        stopping ? ExecuteRobotCommand::Feedback::PHASE_STOPPING :
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
    executor_->request_stop(StopClock::now());
    if (executor_->publishes_cmd_vel()) {
      ros_io_->publish_zero_velocity();
    }
    publish_mode();
    const auto stop = executor_->poll_stop(StopClock::now());
    publish_action_ack(
      "stop", "accepted",
      "emergency_stop;" + stop.detail);
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
    ros_io_->publish_action_ack(
      action, status,
      executor_ ? executor_->backend_name() : "unconfigured", detail);
  }

  void publish_bt_status(const CommandTreeResult & result)
  {
    const std::string command_id = active_goal_ ?
      active_goal_->get_goal()->command.command_id : "";
    ros_io_->publish_bt_status(result, command_id);
  }

  void publish_mode()
  {
    ros_io_->publish_mode(executor_->mode_name());
  }

  void publish_diagnostics()
  {
    if (!ros_io_ || !executor_) {
      return;
    }
    ControllerOutput output;
    {
      std::lock_guard<std::mutex> lock(diagnostics_mutex_);
      output = diagnostic_output_;
    }
    // 回调组可以并行运行；传给纯函数的内容全部来自锁保护快照或只读配置。
    ros_io_->publish_diagnostics({
      get_fully_qualified_name(), get_current_state().label(), executor_plugin_,
      executor_backend_, SimulationController::mode_name(output.mode), output.reason,
      action_active_, output.sensor_stale, output.safety_stopped});
    // readiness 使用周期心跳判断进程是否仍存活，不能只依赖启动时的一次 latched 状态。
    ros_io_->publish_health(
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
    if (ros_io_) {
      ros_io_->reset();
    }
    ros_io_.reset();
    active_goal_.reset();
    action_active_ = false;
    if (action_runtime_) {
      action_runtime_->reset();
    }
    action_runtime_.reset();
    executor_backend_.clear();
    behavior_tree_.reset();
  }

  void control_tick()
  {
    const double now = now_seconds();
    const auto output = executor_->step(now);
    {
      std::lock_guard<std::mutex> lock(diagnostics_mutex_);
      diagnostic_output_ = output;
    }
    const bool action_was_active = action_active_.load();
    const bool action_stopped = update_active_action(output, now);
    if (should_publish_executor_cmd_vel(
        executor_->publishes_cmd_vel(), action_was_active, action_stopped,
        executor_->mode_name()))
    {
      ros_io_->publish_velocity(
        action_stopped ? 0.0 : output.velocity.linear_x,
        action_stopped ? 0.0 : output.velocity.angular_z);
    }
    ros_io_->publish_state(output);
  }

  pluginlib::ClassLoader<RobotExecutor> executor_loader_;
  std::shared_ptr<RobotExecutor> executor_;
  std::string executor_plugin_;
  std::string executor_backend_;
  std::unique_ptr<SimulationRosIo> ros_io_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;
  double action_timeout_s_{12.0};
  rclcpp_action::Server<ExecuteRobotCommand>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_goal_;
  bool use_behavior_tree_{true};
  std::unique_ptr<CommandBehaviorTree> behavior_tree_;
  std::unique_ptr<ActiveActionRuntime> action_runtime_;
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
