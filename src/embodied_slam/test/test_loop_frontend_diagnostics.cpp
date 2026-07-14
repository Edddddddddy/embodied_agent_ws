#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "embodied_slam/loop_frontend_diagnostics.hpp"

namespace
{

using embodied_slam::LoopCandidateScan;
using embodied_slam::LoopMatcherEventType;

TEST(LoopFrontendDiagnostics, ParsesCoarseFineAndRejectedMatcherEvents)
{
  const auto coarse = embodied_slam::parseLoopMatcherEvent(
    "COARSE RESPONSE: 0.28 (> 0.3)\n            var: 1.2,  4.5 (< 3.0)");
  EXPECT_EQ(coarse.type, LoopMatcherEventType::kCoarseCheck);
  ASSERT_TRUE(coarse.response.has_value());
  EXPECT_DOUBLE_EQ(*coarse.response, 0.28);
  EXPECT_DOUBLE_EQ(*coarse.response_threshold, 0.3);
  EXPECT_DOUBLE_EQ(*coarse.variance_x, 1.2);
  EXPECT_DOUBLE_EQ(*coarse.variance_y, 4.5);
  EXPECT_DOUBLE_EQ(*coarse.variance_threshold, 3.0);

  const auto fine = embodied_slam::parseLoopMatcherEvent("FINE RESPONSE: 0.41 (>0.4)");
  EXPECT_EQ(fine.type, LoopMatcherEventType::kFineCheck);
  EXPECT_DOUBLE_EQ(*fine.response, 0.41);
  EXPECT_DOUBLE_EQ(*fine.response_threshold, 0.4);
  EXPECT_EQ(
    embodied_slam::parseLoopMatcherEvent("REJECTED!").type,
    LoopMatcherEventType::kRejected);
}

TEST(LoopFrontendDiagnostics, DistinguishesNoGeometryFromNearLinkedExclusion)
{
  const std::vector<LoopCandidateScan> far_scans{
    {0, 10.0, 0.0, false, false},
    {1, 0.0, 0.0, true, true},
  };
  const auto no_geometry = embodied_slam::summarizeLoopCandidateTopology(
    far_scans, 0.0, 0.0, 2.0, 1);
  EXPECT_EQ(no_geometry.primary_reason, "no_geometric_neighbor");

  const std::vector<LoopCandidateScan> linked_scans{
    {0, 0.2, 0.0, true, false},
    {1, 0.0, 0.0, true, true},
  };
  const auto linked = embodied_slam::summarizeLoopCandidateTopology(
    linked_scans, 0.0, 0.0, 2.0, 1);
  EXPECT_EQ(linked.primary_reason, "all_geometric_neighbors_near_linked");
  EXPECT_EQ(linked.near_linked_count, 1U);
  EXPECT_EQ(linked.eligible_unlinked_count, 0U);
}

TEST(LoopFrontendDiagnostics, MirrorsFarTerminatedCandidateChainRule)
{
  const std::vector<LoopCandidateScan> scans{
    {0, 0.2, 0.0, false, false},
    {1, 0.4, 0.0, false, false},
    {2, 4.0, 0.0, false, false},
    {3, 0.0, 0.0, true, true},
  };
  const auto result = embodied_slam::summarizeLoopCandidateTopology(
    scans, 0.0, 0.0, 2.0, 2);
  EXPECT_EQ(result.qualifying_chain_count, 1U);
  EXPECT_EQ(result.maximum_eligible_chain_size, 2U);
  EXPECT_EQ(result.primary_reason, "matcher_candidate_available");
}

TEST(LoopFrontendDiagnostics, ExplainsChainThatNeverReachesMinimum)
{
  const std::vector<LoopCandidateScan> scans{
    {0, 0.2, 0.0, false, false},
    {1, 4.0, 0.0, false, false},
    {2, 0.0, 0.0, true, true},
  };
  const auto result = embodied_slam::summarizeLoopCandidateTopology(
    scans, 0.0, 0.0, 2.0, 2);
  EXPECT_EQ(result.qualifying_chain_count, 0U);
  EXPECT_EQ(result.maximum_eligible_chain_size, 1U);
  EXPECT_EQ(result.primary_reason, "eligible_chain_below_minimum");
}

}  // namespace
