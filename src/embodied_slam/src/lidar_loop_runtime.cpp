#include "embodied_slam/lidar_loop_runtime.hpp"

#include <cmath>
#include <stdexcept>

namespace embodied_slam
{

LiveLidarLoopDetector::LiveLidarLoopDetector(const LiveLidarLoopDetectorConfig & config)
: config_(config), index_(config.index)
{
  if (config_.top_k == 0 || !std::isfinite(config_.sample_interval_s) ||
    config_.sample_interval_s < 0.0)
  {
    throw std::invalid_argument("invalid live lidar loop detector configuration");
  }
}

std::optional<LiveLidarLoopBatch> LiveLidarLoopDetector::ingest(
  const double stamp_s, const std::vector<LidarPoint2D> & points)
{
  if (!std::isfinite(stamp_s)) {
    throw std::invalid_argument("lidar scan timestamp must be finite");
  }
  if (last_input_stamp_s_ && stamp_s + 1e-9 < *last_input_stamp_s_) {
    throw std::invalid_argument("lidar scan timestamps must be monotonic");
  }
  last_input_stamp_s_ = stamp_s;

  if (last_sample_stamp_s_ &&
    stamp_s - *last_sample_stamp_s_ + 1e-9 < config_.sample_interval_s)
  {
    return std::nullopt;
  }

  const auto descriptor = makePolarScanDescriptor(points, config_.descriptor);
  if (!descriptor.valid()) {
    return std::nullopt;
  }

  const int query_id = next_scan_id_++;
  // 必须先查询再入库，保证当前帧永远不会匹配自身；候选仍只是 shadow evidence，
  // 是否形成图约束由后续几何验证层决定。
  auto candidates = index_.query(stamp_s, descriptor, config_.top_k);
  index_.add(query_id, stamp_s, descriptor);
  last_sample_stamp_s_ = stamp_s;
  return LiveLidarLoopBatch{query_id, stamp_s, index_.size(), std::move(candidates)};
}

std::size_t LiveLidarLoopDetector::indexedScans() const
{
  return index_.size();
}

}  // namespace embodied_slam
