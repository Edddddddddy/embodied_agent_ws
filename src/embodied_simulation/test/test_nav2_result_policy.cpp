#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "embodied_simulation/nav2_result_policy.hpp"

namespace embodied_simulation
{
namespace
{

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

TEST(Nav2ResultPolicyTest, RejectsProtocolSuccessWhenAnyWaypointWasMissed)
{
  auto result = std::make_shared<FollowWaypoints::Result>();
  result->error_code = FollowWaypoints::Result::NONE;
  nav2_msgs::msg::MissedWaypoint missed;
  missed.index = 1U;
  missed.error_code = 304U;
  result->missed_waypoints.push_back(missed);

  const auto outcome = evaluate_follow_waypoints_result(
    {"kitchen", "office"}, rclcpp_action::ResultCode::SUCCEEDED, result.get());

  EXPECT_EQ(outcome.state, ActionExecutionState::kBlocked);
  EXPECT_NE(outcome.detail.find("missed_waypoints=1"), std::string::npos);
  EXPECT_NE(outcome.detail.find("missed_detail=1:304"), std::string::npos);
}

TEST(Nav2ResultPolicyTest, AcceptsOnlyCompleteSuccessfulRoute)
{
  auto result = std::make_shared<FollowWaypoints::Result>();
  result->error_code = FollowWaypoints::Result::NONE;

  const auto outcome = evaluate_follow_waypoints_result(
    {"kitchen", "office"}, rclcpp_action::ResultCode::SUCCEEDED, result.get());

  EXPECT_EQ(outcome.state, ActionExecutionState::kSucceeded);
  EXPECT_NE(outcome.detail.find("missed_waypoints=0"), std::string::npos);
}

TEST(Nav2ResultPolicyTest, RejectsProtocolSuccessWithBusinessErrorCode)
{
  auto result = std::make_shared<FollowWaypoints::Result>();
  result->error_code = FollowWaypoints::Result::TASK_EXECUTOR_FAILED;

  const auto outcome = evaluate_follow_waypoints_result(
    {"kitchen"}, rclcpp_action::ResultCode::SUCCEEDED, result.get());

  EXPECT_EQ(outcome.state, ActionExecutionState::kBlocked);
  EXPECT_NE(outcome.detail.find("error_code=601"), std::string::npos);
}

TEST(Nav2ResultPolicyTest, PreservesExplicitCancellation)
{
  auto result = std::make_shared<FollowWaypoints::Result>();

  const auto outcome = evaluate_follow_waypoints_result(
    {"kitchen"}, rclcpp_action::ResultCode::CANCELED, result.get());

  EXPECT_EQ(outcome.state, ActionExecutionState::kCanceled);
}

}  // namespace
}  // namespace embodied_simulation
