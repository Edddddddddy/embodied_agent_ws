#include "embodied_slam/lidar_loop_temporal_consistency.hpp"

#include <cmath>
#include <stdexcept>

namespace embodied_slam
{
namespace
{

constexpr double kNanosecondsToSeconds = 1.0e-9;

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finitePose(const Pose2d & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

void validateConfig(const LidarLoopTemporalConsistencyConfig & config)
{
  if (config.minimum_confirmations == 0U ||
    !std::isfinite(config.maximum_query_gap_s) || config.maximum_query_gap_s <= 0.0 ||
    !std::isfinite(config.maximum_pair_age_delta_s) ||
    config.maximum_pair_age_delta_s < 0.0 ||
    !std::isfinite(config.maximum_translation_delta_m) ||
    config.maximum_translation_delta_m < 0.0 ||
    !std::isfinite(config.maximum_yaw_delta_rad) || config.maximum_yaw_delta_rad < 0.0)
  {
    throw std::invalid_argument("invalid lidar loop temporal consistency configuration");
  }
}

}  // namespace

LidarLoopTemporalConsistency::LidarLoopTemporalConsistency(
  const LidarLoopTemporalConsistencyConfig & config)
: config_(config)
{
  validateConfig(config_);
}

void LidarLoopTemporalConsistency::startTrack(
  const LidarLoopTemporalObservation & observation)
{
  previous_ = observation;
  has_previous_ = true;
  confirmation_count_ = 1U;
}

LidarLoopTemporalDecision LidarLoopTemporalConsistency::observe(
  const LidarLoopTemporalObservation & observation)
{
  LidarLoopTemporalDecision decision;
  decision.confirmation_required = config_.minimum_confirmations;
  if (observation.query_stamp_ns <= 0 || observation.candidate_stamp_ns <= 0 ||
    observation.candidate_stamp_ns >= observation.query_stamp_ns ||
    observation.query_id == observation.candidate_id || !finitePose(observation.target_to_source))
  {
    decision.reason = "temporal_invalid_observation";
    return decision;
  }

  if (!has_previous_) {
    startTrack(observation);
    decision.confirmation_count = confirmation_count_;
    decision.approved = confirmation_count_ >= config_.minimum_confirmations;
    decision.reason = decision.approved ?
      "temporal_consistency_confirmed" : "temporal_confirmation_pending";
    return decision;
  }

  if (observation.query_stamp_ns <= previous_.query_stamp_ns) {
    // 迟到/重复消息不能倒退当前轨迹，也不能被计入连续确认。
    decision.confirmation_count = confirmation_count_;
    decision.reason = "temporal_non_monotonic_query";
    return decision;
  }

  decision.query_gap_s = static_cast<double>(
    observation.query_stamp_ns - previous_.query_stamp_ns) * kNanosecondsToSeconds;
  const auto current_pair_age = observation.query_stamp_ns - observation.candidate_stamp_ns;
  const auto previous_pair_age = previous_.query_stamp_ns - previous_.candidate_stamp_ns;
  decision.pair_age_delta_s = std::abs(
    static_cast<double>(current_pair_age - previous_pair_age) * kNanosecondsToSeconds);
  decision.translation_delta_m = std::hypot(
    observation.target_to_source.x - previous_.target_to_source.x,
    observation.target_to_source.y - previous_.target_to_source.y);
  decision.yaw_delta_rad = std::abs(wrapAngle(
    observation.target_to_source.yaw - previous_.target_to_source.yaw));

  std::string reset_reason;
  if (decision.query_gap_s > config_.maximum_query_gap_s) {
    reset_reason = "temporal_query_gap_reset";
  } else if (decision.pair_age_delta_s > config_.maximum_pair_age_delta_s) {
    reset_reason = "temporal_pair_age_reset";
  } else if (decision.translation_delta_m > config_.maximum_translation_delta_m ||
    decision.yaw_delta_rad > config_.maximum_yaw_delta_rad)
  {
    reset_reason = "temporal_transform_reset";
  }

  if (!reset_reason.empty()) {
    startTrack(observation);
    decision.confirmation_count = confirmation_count_;
    decision.reason = reset_reason;
    return decision;
  }

  previous_ = observation;
  ++confirmation_count_;
  decision.confirmation_count = confirmation_count_;
  decision.approved = confirmation_count_ >= config_.minimum_confirmations;
  decision.reason = decision.approved ?
    "temporal_consistency_confirmed" : "temporal_confirmation_pending";
  return decision;
}

void LidarLoopTemporalConsistency::reset()
{
  has_previous_ = false;
  confirmation_count_ = 0U;
  previous_ = {};
}

}  // namespace embodied_slam
