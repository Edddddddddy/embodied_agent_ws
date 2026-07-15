#include "embodied_slam/lidar_loop_constraint_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace embodied_slam
{
namespace
{

bool finitePose(const Pose2d & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool finiteMetrics(const LidarLoopConstraintInput & input)
{
  return std::isfinite(input.descriptor_similarity) && std::isfinite(input.inlier_ratio) &&
         std::isfinite(input.overlap_ratio) && std::isfinite(input.rmse_m) &&
         std::isfinite(input.observability_ratio);
}

void validateConfig(const LidarLoopConstraintGateConfig & config)
{
  if (config.minimum_submap_scans == 0U || config.minimum_correspondences < 3U ||
    config.maximum_history == 0U || config.minimum_commit_query_separation < 0 ||
    !std::isfinite(config.minimum_descriptor_similarity) ||
    config.minimum_descriptor_similarity < 0.0 || config.minimum_descriptor_similarity > 1.0 ||
    !std::isfinite(config.minimum_inlier_ratio) || config.minimum_inlier_ratio < 0.0 ||
    config.minimum_inlier_ratio > 1.0 || !std::isfinite(config.minimum_overlap_ratio) ||
    config.minimum_overlap_ratio < 0.0 || config.minimum_overlap_ratio > 1.0 ||
    !std::isfinite(config.maximum_rmse_m) || config.maximum_rmse_m <= 0.0 ||
    !std::isfinite(config.minimum_observability_ratio) ||
    config.minimum_observability_ratio < 0.0 || config.minimum_observability_ratio > 1.0 ||
    !std::isfinite(config.translation_variance) || config.translation_variance <= 0.0 ||
    !std::isfinite(config.yaw_variance) || config.yaw_variance <= 0.0)
  {
    throw std::invalid_argument("invalid lidar loop constraint gate configuration");
  }
}

}  // namespace

LidarLoopConstraintGate::LidarLoopConstraintGate(
  const LidarLoopConstraintGateConfig & config)
: config_(config), temporal_consistency_(config.temporal)
{
  validateConfig(config_);
}

bool LidarLoopConstraintGate::PairKey::operator==(const PairKey & other) const
{
  return query_stamp_ns == other.query_stamp_ns &&
         candidate_stamp_ns == other.candidate_stamp_ns;
}

std::size_t LidarLoopConstraintGate::PairKeyHash::operator()(const PairKey & key) const
{
  const auto query = static_cast<std::uint64_t>(key.query_stamp_ns);
  const auto candidate = static_cast<std::uint64_t>(key.candidate_stamp_ns);
  return static_cast<std::size_t>(
    query * 0x9e3779b185ebca87ULL ^ (candidate + 0x85ebca77c2b2ae63ULL));
}

std::string LidarLoopConstraintGate::validate(const LidarLoopConstraintInput & input) const
{
  if (input.query_stamp_ns <= 0 || input.candidate_stamp_ns <= 0 ||
    input.candidate_stamp_ns >= input.query_stamp_ns || input.query_id == input.candidate_id)
  {
    return "invalid_identity_or_time_order";
  }
  if (!input.available || !input.converged || !input.accepted) {
    return "upstream_geometry_rejected";
  }
  if (input.yaw_ambiguous) {
    return "yaw_ambiguous";
  }
  if (!finitePose(input.target_to_source) || !finiteMetrics(input)) {
    return "non_finite_geometry";
  }
  if (config_.require_scan_to_submap && input.matching_mode != "scan_to_submap") {
    return "matching_mode_not_allowed";
  }
  if (input.query_submap_scans < config_.minimum_submap_scans ||
    input.candidate_submap_scans < config_.minimum_submap_scans)
  {
    return "insufficient_submap_support";
  }
  if (input.correspondences < config_.minimum_correspondences) {
    return "insufficient_correspondences";
  }
  if (input.descriptor_similarity < config_.minimum_descriptor_similarity) {
    return "low_descriptor_similarity";
  }
  if (input.inlier_ratio < config_.minimum_inlier_ratio) {
    return "low_inlier_ratio";
  }
  if (input.overlap_ratio < config_.minimum_overlap_ratio) {
    return "low_bidirectional_overlap";
  }
  if (input.rmse_m > config_.maximum_rmse_m) {
    return "high_rmse";
  }
  if (input.observability_ratio < config_.minimum_observability_ratio) {
    return "degenerate_geometry";
  }
  return "eligible";
}

double LidarLoopConstraintGate::qualityScore(const LidarLoopConstraintInput & input) const
{
  // 分数只用于同一 query 内排序，不把启发式量伪装成概率。重叠和内点率体现
  // 几何支持，RMSE 项抑制“点多但残差大”的重复走廊匹配。
  const double rmse_score = std::clamp(1.0 - input.rmse_m / config_.maximum_rmse_m, 0.0, 1.0);
  return 0.30 * input.descriptor_similarity + 0.25 * input.inlier_ratio +
         0.30 * input.overlap_ratio + 0.15 * rmse_score;
}

void LidarLoopConstraintGate::remember(const PairKey & key)
{
  history_.insert(key);
  history_order_.push_back(key);
  while (history_order_.size() > config_.maximum_history) {
    history_.erase(history_order_.front());
    history_order_.pop_front();
  }
}

std::vector<LidarLoopConstraintDecision> LidarLoopConstraintGate::evaluate(
  const std::vector<LidarLoopConstraintInput> & inputs)
{
  std::vector<LidarLoopConstraintDecision> decisions;
  decisions.reserve(inputs.size());
  std::size_t best_index = std::numeric_limits<std::size_t>::max();
  double best_score = -1.0;

  for (const auto & input : inputs) {
    LidarLoopConstraintDecision decision;
    decision.sequence = next_sequence_++;
    decision.input = input;
    decision.covariance = {
      config_.translation_variance, 0.0, 0.0,
      0.0, config_.translation_variance, 0.0,
      0.0, 0.0, config_.yaw_variance};
    decision.reason = validate(input);
    if (decision.reason == "eligible") {
      const PairKey key{input.query_stamp_ns, input.candidate_stamp_ns};
      if (history_.count(key) > 0U) {
        decision.reason = "duplicate_pair";
      } else {
        decision.quality_score = qualityScore(input);
        remember(key);
        if (decision.quality_score > best_score ||
          (decision.quality_score == best_score &&
          (best_index == std::numeric_limits<std::size_t>::max() ||
          input.rank < decisions[best_index].input.rank)))
        {
          best_score = decision.quality_score;
          best_index = decisions.size();
        }
      }
    }
    decisions.push_back(std::move(decision));
  }

  if (best_index == std::numeric_limits<std::size_t>::max()) {
    return decisions;
  }
  for (std::size_t index = 0U; index < decisions.size(); ++index) {
    if (index != best_index && decisions[index].reason == "eligible") {
      decisions[index].reason = "lower_quality_candidate";
    }
  }
  auto & selected = decisions[best_index];
  selected.temporal = temporal_consistency_.observe({
      selected.input.query_id,
      selected.input.query_stamp_ns,
      selected.input.candidate_id,
      selected.input.candidate_stamp_ns,
      selected.input.target_to_source});
  if (!selected.temporal.approved) {
    selected.reason = selected.temporal.reason;
    return decisions;
  }
  selected.policy_approved = true;
  if (!config_.commit_enabled) {
    selected.reason = "shadow_mode";
    return decisions;
  }
  if (last_commit_query_id_ >= 0 &&
    selected.input.query_id <
    last_commit_query_id_ + config_.minimum_commit_query_separation)
  {
    selected.policy_approved = false;
    selected.reason = "commit_rate_limited";
    return decisions;
  }
  selected.commit_requested = true;
  selected.reason = "commit_requested";
  last_commit_query_id_ = selected.input.query_id;
  return decisions;
}

void LidarLoopConstraintGate::reset()
{
  next_sequence_ = 1U;
  last_commit_query_id_ = -1;
  history_order_.clear();
  history_.clear();
  temporal_consistency_.reset();
}

}  // namespace embodied_slam
