#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "embodied_simulation/robot_executor.hpp"

namespace embodied_simulation
{
namespace
{

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

class DelayedNavigateServer
{
public:
  enum class CancelPolicy
  {
    kAccept,
    kReject,
  };

  explicit DelayedNavigateServer(CancelPolicy cancel_policy = CancelPolicy::kAccept)
  : cancel_policy_(cancel_policy),
    node_(std::make_shared<rclcpp::Node>("delayed_nav2_terminal_server"))
  {
    server_ = rclcpp_action::create_server<NavigateToPose>(
      node_,
      "navigate_to_pose",
      [](const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal>)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const ServerGoalHandlePtr)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          cancel_received_ = true;
        }
        condition_.notify_all();
        return cancel_policy_ == CancelPolicy::kAccept ?
               rclcpp_action::CancelResponse::ACCEPT :
               rclcpp_action::CancelResponse::REJECT;
      },
      [this](const ServerGoalHandlePtr goal_handle)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          goal_handle_ = goal_handle;
        }
        condition_.notify_all();
      });
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});
  }

  ~DelayedNavigateServer()
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  bool wait_for_goal(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return goal_handle_ != nullptr;});
  }

  bool wait_for_cancel(std::chrono::milliseconds timeout = 1s)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return cancel_received_;});
  }

  void complete_canceled()
  {
    const auto handle = goal_handle();
    ASSERT_NE(handle, nullptr);
    auto result = std::make_shared<NavigateToPose::Result>();
    handle->canceled(result);
  }

  void complete_aborted()
  {
    const auto handle = goal_handle();
    ASSERT_NE(handle, nullptr);
    auto result = std::make_shared<NavigateToPose::Result>();
    handle->abort(result);
  }

private:
  ServerGoalHandlePtr goal_handle()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal_handle_;
  }

  CancelPolicy cancel_policy_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::mutex mutex_;
  std::condition_variable condition_;
  ServerGoalHandlePtr goal_handle_;
  bool cancel_received_{false};
};

class Nav2RobotExecutorStopTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // 独立 domain 避免开发机上正在运行的 Nav2 server 抢走测试 goal。
    setenv("ROS_DOMAIN_ID", "222", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static std::shared_ptr<RobotExecutor> make_executor(double stop_timeout_s)
  {
    static pluginlib::ClassLoader<RobotExecutor> loader(
      "embodied_simulation", "embodied_simulation::RobotExecutor");
    auto executor = loader.createSharedInstance(
      "embodied_simulation/Nav2RobotExecutor");
    ControllerConfig config;
    config.stop_timeout_s = stop_timeout_s;
    executor->configure(config);
    return executor;
  }

  static RobotCommand navigate_home()
  {
    RobotCommand command;
    command.action_type = command.NAVIGATE_TO;
    command.target = "home";
    return command;
  }

  static bool wait_until_quiesced(
    const std::shared_ptr<RobotExecutor> & executor,
    std::chrono::milliseconds timeout = 1s)
  {
    const auto deadline = StopClock::now() + timeout;
    while (StopClock::now() < deadline) {
      if (executor->is_quiesced()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return executor->is_quiesced();
  }
};

TEST_F(Nav2RobotExecutorStopTest, CancelAcknowledgementIsNotTerminalCompletion)
{
  DelayedNavigateServer server;
  auto executor = make_executor(2.0);
  ASSERT_TRUE(executor->execute(navigate_home(), 0.0));
  ASSERT_TRUE(server.wait_for_goal());

  const auto requested_at = StopClock::now();
  executor->request_stop(requested_at);
  ASSERT_TRUE(server.wait_for_cancel());

  // fake server 已接受 cancel，但故意延迟 result；STOP 此时必须仍为 STOPPING。
  const auto before_terminal = executor->poll_stop(requested_at + 500ms);
  EXPECT_EQ(before_terminal.state, StopExecutionState::kStopping);
  EXPECT_FALSE(before_terminal.succeeded());
  EXPECT_FALSE(executor->is_quiesced());

  server.complete_canceled();
  ASSERT_TRUE(wait_until_quiesced(executor));
  const auto after_terminal = executor->poll_stop(StopClock::now());
  EXPECT_EQ(after_terminal.state, StopExecutionState::kQuiesced);
  EXPECT_TRUE(after_terminal.succeeded());
}

TEST_F(Nav2RobotExecutorStopTest, MissingTerminalTimesOutFailClosed)
{
  DelayedNavigateServer server;
  auto executor = make_executor(0.05);
  ASSERT_TRUE(executor->execute(navigate_home(), 0.0));
  ASSERT_TRUE(server.wait_for_goal());

  const auto requested_at = StopClock::now();
  executor->request_stop(requested_at);
  ASSERT_TRUE(server.wait_for_cancel());

  const auto timed_out = executor->poll_stop(requested_at + 100ms);
  EXPECT_EQ(timed_out.state, StopExecutionState::kTimedOut);
  EXPECT_FALSE(timed_out.succeeded());
  EXPECT_FALSE(executor->is_quiesced());

  // 即使 STOP 已向上层报告超时，晚到的真实 terminal 仍会解除内部封锁。
  server.complete_canceled();
  EXPECT_TRUE(wait_until_quiesced(executor));
}

TEST_F(Nav2RobotExecutorStopTest, RejectedCancelRequestFailsClosed)
{
  DelayedNavigateServer server(DelayedNavigateServer::CancelPolicy::kReject);
  auto executor = make_executor(2.0);
  ASSERT_TRUE(executor->execute(navigate_home(), 0.0));
  ASSERT_TRUE(server.wait_for_goal());

  executor->request_stop(StopClock::now());
  ASSERT_TRUE(server.wait_for_cancel());

  StopExecutionUpdate update;
  const auto deadline = StopClock::now() + 1s;
  do {
    update = executor->poll_stop(StopClock::now());
    if (update.state == StopExecutionState::kFailed) {
      break;
    }
    std::this_thread::sleep_for(5ms);
  } while (StopClock::now() < deadline);

  EXPECT_EQ(update.state, StopExecutionState::kFailed);
  EXPECT_FALSE(executor->is_quiesced());
  server.complete_aborted();
  EXPECT_TRUE(wait_until_quiesced(executor));
}

}  // namespace
}  // namespace embodied_simulation
