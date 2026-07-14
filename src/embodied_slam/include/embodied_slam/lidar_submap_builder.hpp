#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "embodied_slam/lidar_scan_corpus.hpp"

namespace embodied_slam
{

struct LidarSubmapConfig
{
  std::size_t half_window_scans{1U};
  double maximum_time_delta_s{0.75};
  std::size_t point_stride{2U};
  std::size_t minimum_contributing_scans{2U};
  std::size_t minimum_points{60U};
};

struct LidarSubmap
{
  bool available{false};
  int center_scan_id{0};
  std::vector<int> contributing_scan_ids;
  std::vector<LidarPoint2D> points;
  std::string rejection_reason;
};

// 该类只借用输入 corpus 的生命周期。邻帧通过短时里程计变换到中心扫描坐标系，
// 以增加门框/拐角等上下文；远距离闭环的相对位姿仍由 scan matcher 独立估计。
class LidarSubmapBuilder
{
public:
  LidarSubmapBuilder(
    const std::vector<LidarScanRecord> & scans,
    const std::vector<LidarOdometryRecord> & odometry,
    LidarSubmapConfig config = {});

  LidarSubmap build(int center_scan_id) const;

private:
  const std::vector<LidarScanRecord> & scans_;
  std::unordered_map<int, std::size_t> scan_index_by_id_;
  std::unordered_map<int, LidarOdometryRecord> odometry_by_id_;
  LidarSubmapConfig config_;
};

}  // namespace embodied_slam
