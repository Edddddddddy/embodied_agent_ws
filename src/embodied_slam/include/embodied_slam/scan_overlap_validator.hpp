#pragma once

#include <cstddef>
#include <vector>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct ScanPoint2d
{
  double x{0.0};
  double y{0.0};
};

struct ScanOverlapConfig
{
  double match_distance_m{0.20};
  std::size_t point_stride{2U};
  std::size_t minimum_points{30U};
};

struct ScanOverlapResult
{
  bool available{false};
  double overlap_ratio{0.0};
  std::size_t source_points{0U};
  std::size_t target_points{0U};
  std::size_t source_matches{0U};
  std::size_t target_matches{0U};
};

// 将 target 扫描按 source^-1 * target 的相对位姿变换到 source 坐标系，计算双向近邻重合率。
// 该指标不读取真值，只作为 Karto 已接受约束的第二份几何证据。
ScanOverlapResult evaluateScanOverlap(
  const std::vector<ScanPoint2d> & source,
  const std::vector<ScanPoint2d> & target,
  const Pose2d & source_to_target,
  const ScanOverlapConfig & config = {});

}  // namespace embodied_slam
