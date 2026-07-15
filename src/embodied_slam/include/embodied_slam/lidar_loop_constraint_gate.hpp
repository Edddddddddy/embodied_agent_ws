#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_set>
#include <vector>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct LidarLoopConstraintGateConfig
{
  bool commit_enabled{false};
  bool require_scan_to_submap{true};
  std::size_t minimum_submap_scans{2U};
  std::size_t minimum_correspondences{30U};
  double minimum_descriptor_similarity{0.60};
  double minimum_inlier_ratio{0.35};
  double minimum_overlap_ratio{0.55};
  double maximum_rmse_m{0.15};
  double minimum_observability_ratio{0.005};
  std::int64_t minimum_commit_query_separation{5};
  std::size_t maximum_history{4096U};
  double translation_variance{0.04};
  double yaw_variance{0.04};
};

struct LidarLoopConstraintInput
{
  std::int64_t query_id{0};
  std::int64_t query_stamp_ns{0};
  std::int64_t candidate_id{0};
  std::int64_t candidate_stamp_ns{0};
  std::uint32_t rank{0U};
  bool upstream_shadow_only{true};
  std::string matching_mode;
  bool available{false};
  bool converged{false};
  bool accepted{false};
  bool yaw_ambiguous{false};
  Pose2d target_to_source;
  std::size_t query_submap_scans{0U};
  std::size_t candidate_submap_scans{0U};
  std::size_t correspondences{0U};
  double descriptor_similarity{0.0};
  double inlier_ratio{0.0};
  double overlap_ratio{0.0};
  double rmse_m{0.0};
  double observability_ratio{0.0};
};

struct LidarLoopConstraintDecision
{
  std::uint64_t sequence{0U};
  LidarLoopConstraintInput input;
  bool policy_approved{false};
  bool commit_requested{false};
  std::array<double, 9U> covariance{};
  double quality_score{0.0};
  std::string reason;
};

// 约束门是纯 C++ 深模块：ROS Adapter 只负责消息转换；择优、去重、节流和
// commit/shadow 语义集中在一个接口内，避免不同后端各自复制安全策略。
class LidarLoopConstraintGate
{
public:
  explicit LidarLoopConstraintGate(const LidarLoopConstraintGateConfig & config = {});

  std::vector<LidarLoopConstraintDecision> evaluate(
    const std::vector<LidarLoopConstraintInput> & inputs);
  void reset();

private:
  struct PairKey
  {
    std::int64_t query_stamp_ns{0};
    std::int64_t candidate_stamp_ns{0};
    bool operator==(const PairKey & other) const;
  };

  struct PairKeyHash
  {
    std::size_t operator()(const PairKey & key) const;
  };

  std::string validate(const LidarLoopConstraintInput & input) const;
  double qualityScore(const LidarLoopConstraintInput & input) const;
  void remember(const PairKey & key);

  LidarLoopConstraintGateConfig config_;
  std::uint64_t next_sequence_{1U};
  std::int64_t last_commit_query_id_{-1};
  std::deque<PairKey> history_order_;
  std::unordered_set<PairKey, PairKeyHash> history_;
};

}  // namespace embodied_slam
