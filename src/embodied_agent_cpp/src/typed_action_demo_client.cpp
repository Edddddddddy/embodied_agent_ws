#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/action/execute_robot_command.hpp>
#include <embodied_agent_interfaces/msg/robot_command.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "embodied_agent_cpp/typed_action_client_contract.hpp"

namespace embodied_agent_cpp
{
namespace
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;
using ExecuteRobotCommand = embodied_agent_interfaces::action::ExecuteRobotCommand;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteRobotCommand>;

double parse_double(const std::string & value, const std::string & name)
{
  try {
    std::size_t parsed = 0;
    const double number = std::stod(value, &parsed);
    if (parsed != value.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return number;
  } catch (const std::exception & error) {
    throw std::invalid_argument("invalid " + name + ": " + value + " (" + error.what() + ")");
  }
}

std::uint32_t parse_uint(const std::string & value, const std::string & name)
{
  try {
    std::size_t parsed = 0;
    const unsigned long number = std::stoul(value, &parsed);
    if (parsed != value.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return static_cast<std::uint32_t>(number);
  } catch (const std::exception & error) {
    throw std::invalid_argument("invalid " + name + ": " + value + " (" + error.what() + ")");
  }
}

std::string usage()
{
  return R"(Usage:
  ros2 run embodied_agent_cpp typed_action_demo_client move LINEAR_X DURATION_S [ANGULAR_Z]
  ros2 run embodied_agent_cpp typed_action_demo_client turn ANGULAR_Z DURATION_S
  ros2 run embodied_agent_cpp typed_action_demo_client stop
  ros2 run embodied_agent_cpp typed_action_demo_client navigate_to TARGET [TIMEOUT_S]
  ros2 run embodied_agent_cpp typed_action_demo_client follow_waypoints TARGET... [--loops N] [--timeout S]
  ros2 run embodied_agent_cpp typed_action_demo_client cancel_navigation

Examples:
  ros2 run embodied_agent_cpp typed_action_demo_client move 0.12 1.0
  ros2 run embodied_agent_cpp typed_action_demo_client turn 0.6 2.6
  ros2 run embodied_agent_cpp typed_action_demo_client navigate_to door 120
)";
}

std::string make_command_id(const std::string & action)
{
  std::ostringstream stream;
  stream << "cpp_demo_" << action << "_" << rclcpp::Clock().now().nanoseconds();
  return stream.str();
}

RobotCommand build_command(const std::vector<std::string> & args)
{
  if (args.empty()) {
    throw std::invalid_argument("missing command\n" + usage());
  }

  RobotCommand command;
  command.header.stamp = rclcpp::Clock().now();
  command.source = "typed_action_demo_client";
  command.command_id = make_command_id(args[0]);

  const auto & action = args[0];
  if (action == "move") {
    if (args.size() < 3 || args.size() > 4) {
      throw std::invalid_argument("move requires LINEAR_X DURATION_S [ANGULAR_Z]\n" + usage());
    }
    command.action_type = RobotCommand::MOVE;
    command.linear_x = parse_double(args[1], "linear_x");
    command.duration_s = parse_double(args[2], "duration_s");
    command.angular_z = args.size() == 4 ? parse_double(args[3], "angular_z") : 0.0;
    return command;
  }
  if (action == "turn") {
    if (args.size() != 3) {
      throw std::invalid_argument("turn requires ANGULAR_Z DURATION_S\n" + usage());
    }
    command.action_type = RobotCommand::TURN;
    command.angular_z = parse_double(args[1], "angular_z");
    command.duration_s = parse_double(args[2], "duration_s");
    return command;
  }
  if (action == "stop") {
    if (args.size() != 1) {
      throw std::invalid_argument("stop takes no arguments\n" + usage());
    }
    command.action_type = RobotCommand::STOP;
    return command;
  }
  if (action == "navigate_to") {
    if (args.size() < 2 || args.size() > 3) {
      throw std::invalid_argument("navigate_to requires TARGET [TIMEOUT_S]\n" + usage());
    }
    command.action_type = RobotCommand::NAVIGATE_TO;
    command.target = args[1];
    command.duration_s = args.size() == 3 ? parse_double(args[2], "timeout_s") : 10.0;
    return command;
  }
  if (action == "follow_waypoints") {
    if (args.size() < 2) {
      throw std::invalid_argument("follow_waypoints requires at least one TARGET\n" + usage());
    }
    command.action_type = RobotCommand::FOLLOW_WAYPOINTS;
    command.number_of_loops = 1;
    command.duration_s = 10.0;
    for (std::size_t index = 1; index < args.size(); ++index) {
      if (args[index] == "--loops") {
        if (index + 1 >= args.size()) {
          throw std::invalid_argument("--loops requires a value\n" + usage());
        }
        command.number_of_loops = parse_uint(args[++index], "loops");
      } else if (args[index] == "--timeout") {
        if (index + 1 >= args.size()) {
          throw std::invalid_argument("--timeout requires a value\n" + usage());
        }
        command.duration_s = parse_double(args[++index], "timeout_s");
      } else {
        command.waypoints.push_back(args[index]);
      }
    }
    if (command.waypoints.empty()) {
      throw std::invalid_argument("follow_waypoints requires at least one TARGET\n" + usage());
    }
    return command;
  }
  if (action == "cancel_navigation") {
    if (args.size() != 1) {
      throw std::invalid_argument("cancel_navigation takes no arguments\n" + usage());
    }
    command.action_type = RobotCommand::CANCEL_NAVIGATION;
    return command;
  }

  throw std::invalid_argument("unknown command: " + action + "\n" + usage());
}

const char * result_code_name(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

class TypedActionDemoClient : public rclcpp::Node
{
public:
  explicit TypedActionDemoClient(const RobotCommand & command)
  : Node("typed_action_demo_client"), command_(command)
  {
    client_ = rclcpp_action::create_client<ExecuteRobotCommand>(
      this, "robot/execute_command");
  }

  bool run()
  {
    started_at_ = std::chrono::steady_clock::now();
    expected_outcome_ = declare_parameter<std::string>(
      "expected_outcome", "succeeded");
    cancel_after_s_ = declare_parameter<double>("cancel_after_s", -1.0);
    result_timeout_s_ = declare_parameter<double>("result_timeout_s", 15.0);
    report_path_ = declare_parameter<std::string>("report_path", "");
    const auto server_timeout = std::chrono::seconds(
      declare_parameter<int>("server_timeout_s", 5));
    if (!client_->wait_for_action_server(server_timeout)) {
      RCLCPP_ERROR(get_logger(), "ExecuteRobotCommand action server unavailable");
      complete(
        ActionTerminalOutcome::kServerUnavailable, "SERVER_UNAVAILABLE", false, 0,
        "action_server_unavailable");
      return true;
    }

    ExecuteRobotCommand::Goal goal;
    goal.command = command_;
    rclcpp_action::Client<ExecuteRobotCommand>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "goal rejected by action server");
          complete(
            ActionTerminalOutcome::kGoalRejected, "GOAL_REJECTED", false, 0,
            "goal_rejected");
          return;
        }
        goal_handle_ = handle;
        goal_accepted_ = true;
        RCLCPP_INFO(get_logger(), "goal accepted: command_id=%s", command_.command_id.c_str());
      };
    options.feedback_callback =
      [this](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ExecuteRobotCommand::Feedback> feedback) {
        ++feedback_count_;
        max_progress_ = std::max(max_progress_, static_cast<double>(feedback->progress));
        std::cout << "feedback phase=" << static_cast<int>(feedback->phase)
                  << " progress=" << std::fixed << std::setprecision(2)
                  << feedback->progress
                  << " detail=" << feedback->detail << std::endl;
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & wrapped) {
        if (!wrapped.result) {
          std::cout << "result code=" << result_code_name(wrapped.code)
                    << " success=false status=0 message=missing_action_result"
                    << std::endl;
          complete(
            ActionTerminalOutcome::kFailed, result_code_name(wrapped.code), false, 0,
            "missing_action_result");
          return;
        }
        std::cout << "result code=" << result_code_name(wrapped.code)
                  << " success=" << (wrapped.result->success ? "true" : "false")
                  << " status=" << static_cast<int>(wrapped.result->status)
                  << " message=" << wrapped.result->message << std::endl;
        complete(
          outcome_from_action_status(wrapped.result->success, wrapped.result->status),
          result_code_name(wrapped.code), wrapped.result->success,
          wrapped.result->status, wrapped.result->message);
      };

    client_->async_send_goal(goal, options);
    if (cancel_after_s_ >= 0.0) {
      const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(cancel_after_s_));
      cancel_timer_ = create_wall_timer(delay, [this]() {
          if (finalized_.load() || cancel_requested_) {
            return;
          }
          // DDS 首次发现时 goal response 可能晚于执行开始。隔离验收环境中主动
          // cancel_all_goals，能保证取消请求按发送时刻生效，而不是被响应延迟拖后。
          cancel_requested_ = true;
          client_->async_cancel_all_goals();
          cancel_timer_->cancel();
          RCLCPP_INFO(get_logger(), "cancel requested by demo scenario");
        });
    }
    RCLCPP_INFO(
      get_logger(), "sent %s command_id=%s", command_.source.c_str(),
      command_.command_id.c_str());
    return true;
  }

  std::future<bool> result_future()
  {
    return result_promise_.get_future();
  }

  double result_timeout_s() const
  {
    return result_timeout_s_;
  }

  void mark_client_timeout()
  {
    if (goal_handle_ && !cancel_requested_) {
      cancel_requested_ = true;
      client_->async_cancel_goal(goal_handle_);
    }
    // 客户端超时是通信/调度故障，不得伪装成服务端 STATUS_TIMED_OUT。
    complete(
      ActionTerminalOutcome::kClientTimedOut, "CLIENT_TIMEOUT", false, 0,
      "result_wait_timeout");
  }

private:
  void complete(
    ActionTerminalOutcome outcome,
    const std::string & transport_code,
    bool action_success,
    unsigned int action_status,
    const std::string & message)
  {
    if (finalized_.exchange(true)) {
      return;
    }
    if (cancel_timer_) {
      cancel_timer_->cancel();
    }
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started_at_).count();
    const bool passed = outcome_matches_expectation(outcome, expected_outcome_);
    const nlohmann::json report{
      {"schema_version", 1},
      {"command_id", command_.command_id},
      {"expected_outcome", expected_outcome_},
      {"observed_outcome", outcome_name(outcome)},
      {"transport_code", transport_code},
      {"goal_accepted", goal_accepted_},
      {"cancel_requested", cancel_requested_},
      {"feedback_count", feedback_count_},
      {"max_progress", max_progress_},
      {"action_success", action_success},
      {"action_status", action_status},
      {"message", message},
      {"elapsed_ms", elapsed_ms},
      {"passed", passed},
    };
    // 固定前缀便于 shell/CI 从混合 ROS 日志中稳定提取结构化证据。
    std::cout << "CPP_ACTION_REPORT " << report.dump() << std::endl;
    if (!report_path_.empty()) {
      std::ofstream stream(report_path_);
      if (stream) {
        stream << std::setw(2) << report << std::endl;
      } else {
        RCLCPP_ERROR(get_logger(), "cannot write report_path=%s", report_path_.c_str());
      }
    }
    result_promise_.set_value(passed);
  }

  RobotCommand command_;
  rclcpp_action::Client<ExecuteRobotCommand>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::promise<bool> result_promise_;
  std::atomic_bool finalized_{false};
  std::chrono::steady_clock::time_point started_at_;
  std::string expected_outcome_{"succeeded"};
  std::string report_path_;
  double cancel_after_s_{-1.0};
  double result_timeout_s_{15.0};
  int feedback_count_{0};
  double max_progress_{0.0};
  bool goal_accepted_{false};
  bool cancel_requested_{false};
};

}  // namespace
}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<std::string> command_args(arguments.begin() + 1, arguments.end());
    const auto command = embodied_agent_cpp::build_command(command_args);
    auto node = std::make_shared<embodied_agent_cpp::TypedActionDemoClient>(command);
    auto result = node->result_future();
    if (!node->run()) {
      rclcpp::shutdown();
      return 1;
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(node->result_timeout_s());
    while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(50)) !=
      std::future_status::ready)
    {
      executor.spin_some();
      if (std::chrono::steady_clock::now() >= deadline) {
        node->mark_client_timeout();
      }
    }
    const bool result_ready = result.valid() &&
      result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
    // SIGINT/ROS shutdown 时 future 可能尚未完成，不能在退出路径再次阻塞 get()。
    const bool ok = result_ready && result.get();
    rclcpp::shutdown();
    return ok ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
}
