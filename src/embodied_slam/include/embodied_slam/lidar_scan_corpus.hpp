#pragma once

#include <string>
#include <vector>

#include "embodied_slam/lidar_loop_descriptor.hpp"

namespace embodied_slam
{

struct LidarScanRecord
{
  int scan_id{0};
  double stamp_s{0.0};
  std::vector<LidarPoint2D> points;
};

// 候选检索和几何配准必须读取完全相同的扫描语料；集中解析可避免两个 CLI
// 对时间单调性、点数量或尾随字段产生不一致解释。
std::vector<LidarScanRecord> loadLidarScanCorpus(const std::string & path);

}  // namespace embodied_slam
