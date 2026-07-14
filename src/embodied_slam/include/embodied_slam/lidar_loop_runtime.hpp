#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "embodied_slam/lidar_loop_descriptor.hpp"

namespace embodied_slam
{

struct LiveLidarLoopDetectorConfig
{
  PolarDescriptorConfig descriptor;
  LoopCandidateIndexConfig index;
  std::size_t top_k{10};
  double sample_interval_s{0.5};
};

struct LiveLidarLoopBatch
{
  int query_id{0};
  double query_stamp_s{0.0};
  std::size_t indexed_scans{0};
  std::vector<LidarLoopCandidate> candidates;
};

// 在线节点只依赖这个领域对象：它统一采样、描述子构建和索引顺序，避免 ROS 回调
// 自行拼装状态后出现“当前帧先入库、再把自己检索为回环”的数据泄漏。
class LiveLidarLoopDetector
{
public:
  explicit LiveLidarLoopDetector(const LiveLidarLoopDetectorConfig & config = {});

  std::optional<LiveLidarLoopBatch> ingest(
    double stamp_s, const std::vector<LidarPoint2D> & points);
  std::size_t indexedScans() const;

private:
  LiveLidarLoopDetectorConfig config_;
  LidarLoopCandidateIndex index_;
  int next_scan_id_{0};
  std::optional<double> last_input_stamp_s_;
  std::optional<double> last_sample_stamp_s_;
};

}  // namespace embodied_slam
