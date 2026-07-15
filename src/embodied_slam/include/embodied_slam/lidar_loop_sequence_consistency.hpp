#pragma once

#include <cstddef>
#include <vector>

#include "embodied_slam/lidar_loop_temporal_consistency.hpp"

namespace embodied_slam
{

struct LidarLoopSequenceConsistencyConfig
{
  std::size_t minimum_confirmations{3U};
  std::size_t maximum_hypotheses{64U};
  double maximum_query_gap_s{2.0};
  double maximum_pair_age_delta_s{0.25};
  double maximum_translation_delta_m{0.35};
  double maximum_yaw_delta_rad{0.20};
};

// SeqSLAM 风格的多假设门控：同一 query 的 Top-K 候选并行延伸各自轨迹，
// 避免单帧最高分的错误候选抢占唯一时序状态。它不读取真值，也不负责写图。
class LidarLoopSequenceConsistency
{
public:
  explicit LidarLoopSequenceConsistency(
    const LidarLoopSequenceConsistencyConfig & config = {});

  std::vector<LidarLoopTemporalDecision> observeBatch(
    const std::vector<LidarLoopTemporalObservation> & observations);
  void reset();

private:
  struct Hypothesis
  {
    LidarLoopTemporalObservation observation;
    std::size_t confirmation_count{1U};
  };

  LidarLoopSequenceConsistencyConfig config_;
  std::vector<Hypothesis> hypotheses_;
  std::int64_t latest_query_stamp_ns_{0};
};

}  // namespace embodied_slam
