#include <gtest/gtest.h>

#include <rmw/types.h>

#include "embodied_agent_middleware/qos_profiles.hpp"

namespace
{

using namespace embodied_agent_middleware;

TEST(QosProfilesTest, CommandIsReliableVolatileAndNeverZeroDepth)
{
  const auto profile = command_qos(0).get_rmw_qos_profile();
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 1U);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfilesTest, StateIsLatchedButEventsAreNot)
{
  const auto state = state_qos().get_rmw_qos_profile();
  const auto event = event_qos().get_rmw_qos_profile();
  EXPECT_EQ(state.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  EXPECT_EQ(event.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  EXPECT_EQ(state.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
}

TEST(QosProfilesTest, SensorAndAudioPreferFreshFrames)
{
  const auto sensor = sensor_qos().get_rmw_qos_profile();
  const auto audio = audio_qos().get_rmw_qos_profile();
  EXPECT_EQ(sensor.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(audio.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(sensor.depth, 5U);
  EXPECT_EQ(audio.depth, 5U);
}

}  // namespace
