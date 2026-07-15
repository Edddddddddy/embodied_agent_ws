#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct LidarLoopTemporalConsistencyConfig
{
  std::size_t minimum_confirmations{4U};
  double maximum_query_gap_s{2.0};
  double maximum_pair_age_delta_s{1.25};
  double maximum_translation_delta_m{0.55};
  double maximum_yaw_delta_rad{0.35};
};

struct LidarLoopTemporalObservation
{
  std::int64_t query_id{0};
  std::int64_t query_stamp_ns{0};
  std::int64_t candidate_id{0};
  std::int64_t candidate_stamp_ns{0};
  Pose2d target_to_source;
};

struct LidarLoopTemporalDecision
{
  bool approved{false};
  std::size_t confirmation_count{0U};
  std::size_t confirmation_required{0U};
  double query_gap_s{0.0};
  double pair_age_delta_s{0.0};
  double translation_delta_m{0.0};
  double yaw_delta_rad{0.0};
  std::string reason;
};

// 多帧门控只关心时序一致性，不知道 ROS 消息、ICP 分数或后端实现。
// 这样在线门控和离线真实数据消融可以共用同一份 C++ 策略，避免评测脚本复制算法。
class LidarLoopTemporalConsistency
{
public:
  explicit LidarLoopTemporalConsistency(
    const LidarLoopTemporalConsistencyConfig & config = {});

  LidarLoopTemporalDecision observe(const LidarLoopTemporalObservation & observation);
  void reset();

private:
  void startTrack(const LidarLoopTemporalObservation & observation);

  LidarLoopTemporalConsistencyConfig config_;
  bool has_previous_{false};
  std::size_t confirmation_count_{0U};
  LidarLoopTemporalObservation previous_;
};

}  // namespace embodied_slam
