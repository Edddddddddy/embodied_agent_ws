#include "embodied_slam/lidar_loop_sequence_consistency.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace embodied_slam
{
namespace
{

constexpr double kNanosecondsToSeconds = 1.0e-9;

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finitePose(const Pose2d & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

void validateConfig(
  const LidarLoopSequenceConsistencyConfig & config)
{
  if (config.maximum_hypotheses == 0U || config.minimum_confirmations == 0U ||
    !std::isfinite(config.maximum_query_gap_s) || config.maximum_query_gap_s <= 0.0 ||
    !std::isfinite(config.maximum_pair_age_delta_s) ||
    config.maximum_pair_age_delta_s < 0.0 ||
    !std::isfinite(config.maximum_translation_delta_m) ||
    config.maximum_translation_delta_m < 0.0 ||
    !std::isfinite(config.maximum_yaw_delta_rad) || config.maximum_yaw_delta_rad < 0.0)
  {
    throw std::invalid_argument("invalid lidar loop sequence consistency configuration");
  }
}

bool validObservation(const LidarLoopTemporalObservation & observation)
{
  return observation.query_stamp_ns > 0 && observation.candidate_stamp_ns > 0 &&
         observation.candidate_stamp_ns < observation.query_stamp_ns &&
         observation.query_id != observation.candidate_id &&
         finitePose(observation.target_to_source);
}

}  // namespace

LidarLoopSequenceConsistency::LidarLoopSequenceConsistency(
  const LidarLoopSequenceConsistencyConfig & config)
: config_(config)
{
  validateConfig(config_);
}

std::vector<LidarLoopTemporalDecision> LidarLoopSequenceConsistency::observeBatch(
  const std::vector<LidarLoopTemporalObservation> & observations)
{
  std::vector<LidarLoopTemporalDecision> decisions(observations.size());
  if (observations.empty()) {
    return decisions;
  }

  const auto query_stamp_ns = observations.front().query_stamp_ns;
  if (std::any_of(
      observations.begin(), observations.end(),
      [query_stamp_ns](const auto & observation) {
        return observation.query_stamp_ns != query_stamp_ns;
      }))
  {
    throw std::invalid_argument("one sequence batch must share one query timestamp");
  }

  for (auto & decision : decisions) {
    decision.confirmation_required = config_.minimum_confirmations;
  }
  if (query_stamp_ns <= latest_query_stamp_ns_) {
    for (auto & decision : decisions) {
      decision.reason = "sequence_non_monotonic_query";
    }
    return decisions;
  }

  const auto still_recent = [this, query_stamp_ns](const Hypothesis & hypothesis) {
      return static_cast<double>(query_stamp_ns - hypothesis.observation.query_stamp_ns) *
             kNanosecondsToSeconds <= config_.maximum_query_gap_s;
    };
  hypotheses_.erase(
    std::remove_if(
      hypotheses_.begin(), hypotheses_.end(),
      [&still_recent](const auto & hypothesis) {return !still_recent(hypothesis);}),
    hypotheses_.end());

  std::vector<Hypothesis> extended;
  extended.reserve(observations.size());
  for (std::size_t index = 0U; index < observations.size(); ++index) {
    const auto & observation = observations[index];
    auto & decision = decisions[index];
    if (!validObservation(observation)) {
      decision.reason = "sequence_invalid_observation";
      continue;
    }

    const Hypothesis * best = nullptr;
    double best_residual = std::numeric_limits<double>::infinity();
    for (const auto & hypothesis : hypotheses_) {
      const auto & previous = hypothesis.observation;
      if (observation.candidate_stamp_ns <= previous.candidate_stamp_ns) {
        continue;
      }
      const double query_gap_s = static_cast<double>(
        observation.query_stamp_ns - previous.query_stamp_ns) * kNanosecondsToSeconds;
      const auto pair_age = observation.query_stamp_ns - observation.candidate_stamp_ns;
      const auto previous_pair_age = previous.query_stamp_ns - previous.candidate_stamp_ns;
      const double pair_age_delta_s = std::abs(
        static_cast<double>(pair_age - previous_pair_age) * kNanosecondsToSeconds);
      const double translation_delta_m = std::hypot(
        observation.target_to_source.x - previous.target_to_source.x,
        observation.target_to_source.y - previous.target_to_source.y);
      const double yaw_delta_rad = std::abs(wrapAngle(
        observation.target_to_source.yaw - previous.target_to_source.yaw));
      if (query_gap_s > config_.maximum_query_gap_s ||
        pair_age_delta_s > config_.maximum_pair_age_delta_s ||
        translation_delta_m > config_.maximum_translation_delta_m ||
        yaw_delta_rad > config_.maximum_yaw_delta_rad)
      {
        continue;
      }
      const double residual = pair_age_delta_s + translation_delta_m + yaw_delta_rad;
      if (best == nullptr || hypothesis.confirmation_count > best->confirmation_count ||
        (hypothesis.confirmation_count == best->confirmation_count && residual < best_residual))
      {
        best = &hypothesis;
        best_residual = residual;
        decision.query_gap_s = query_gap_s;
        decision.pair_age_delta_s = pair_age_delta_s;
        decision.translation_delta_m = translation_delta_m;
        decision.yaw_delta_rad = yaw_delta_rad;
      }
    }

    const std::size_t confirmations = best == nullptr ? 1U : best->confirmation_count + 1U;
    decision.confirmation_count = confirmations;
    decision.approved = confirmations >= config_.minimum_confirmations;
    decision.reason = decision.approved ?
      "sequence_consistency_confirmed" :
      (best == nullptr ? "sequence_hypothesis_started" : "sequence_confirmation_pending");
    extended.push_back({observation, confirmations});
  }

  // 保留近期旧轨迹允许短暂漏检；再加入当前 Top-K 分支。按确认长度优先裁剪，
  // 防止重复走廊令假设数随时间无界增长。
  hypotheses_.insert(hypotheses_.end(), extended.begin(), extended.end());
  std::stable_sort(
    hypotheses_.begin(), hypotheses_.end(),
    [](const Hypothesis & left, const Hypothesis & right) {
      if (left.confirmation_count != right.confirmation_count) {
        return left.confirmation_count > right.confirmation_count;
      }
      if (left.observation.query_stamp_ns != right.observation.query_stamp_ns) {
        return left.observation.query_stamp_ns > right.observation.query_stamp_ns;
      }
      return left.observation.candidate_stamp_ns < right.observation.candidate_stamp_ns;
    });
  if (hypotheses_.size() > config_.maximum_hypotheses) {
    hypotheses_.resize(config_.maximum_hypotheses);
  }
  latest_query_stamp_ns_ = query_stamp_ns;
  return decisions;
}

void LidarLoopSequenceConsistency::reset()
{
  hypotheses_.clear();
  latest_query_stamp_ns_ = 0;
}

}  // namespace embodied_slam
