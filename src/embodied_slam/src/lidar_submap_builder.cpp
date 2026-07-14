#include "embodied_slam/lidar_submap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace embodied_slam
{
namespace
{

Pose2d relativePose(const Pose2d & center, const Pose2d & neighbor)
{
  const double world_x = neighbor.x - center.x;
  const double world_y = neighbor.y - center.y;
  const double cosine = std::cos(center.yaw);
  const double sine = std::sin(center.yaw);
  return {
    cosine * world_x + sine * world_y,
    -sine * world_x + cosine * world_y,
    normalize_angle(neighbor.yaw - center.yaw)};
}

LidarPoint2D transformPoint(const LidarPoint2D & point, const Pose2d & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {
    cosine * point.x - sine * point.y + pose.x,
    sine * point.x + cosine * point.y + pose.y};
}

}  // namespace

LidarSubmapBuilder::LidarSubmapBuilder(
  const std::vector<LidarScanRecord> & scans,
  const std::vector<LidarOdometryRecord> & odometry,
  LidarSubmapConfig config)
: scans_(scans), config_(config)
{
  if (scans.empty() || odometry.empty() || config_.point_stride == 0U ||
    config_.minimum_contributing_scans == 0U || config_.minimum_points < 3U ||
    !std::isfinite(config_.maximum_time_delta_s) || config_.maximum_time_delta_s <= 0.0)
  {
    throw std::invalid_argument("invalid lidar submap configuration or empty corpus");
  }
  for (std::size_t index = 0U; index < scans_.size(); ++index) {
    if (!scan_index_by_id_.emplace(scans_[index].scan_id, index).second) {
      throw std::invalid_argument("duplicate scan id in submap corpus");
    }
  }
  for (const auto & row : odometry) {
    if (!odometry_by_id_.emplace(row.scan_id, row).second) {
      throw std::invalid_argument("duplicate odometry id in submap corpus");
    }
  }
}

LidarSubmap LidarSubmapBuilder::build(const int center_scan_id) const
{
  LidarSubmap result;
  result.center_scan_id = center_scan_id;
  const auto center_index = scan_index_by_id_.find(center_scan_id);
  const auto center_odometry = odometry_by_id_.find(center_scan_id);
  if (center_index == scan_index_by_id_.end() || center_odometry == odometry_by_id_.end()) {
    result.rejection_reason = "missing_center_scan_or_odometry";
    return result;
  }
  const std::size_t first = center_index->second > config_.half_window_scans ?
    center_index->second - config_.half_window_scans : 0U;
  const std::size_t last = std::min(
    scans_.size() - 1U, center_index->second + config_.half_window_scans);
  for (std::size_t index = first; index <= last; ++index) {
    const auto & scan = scans_[index];
    if (std::abs(scan.stamp_s - scans_[center_index->second].stamp_s) >
      config_.maximum_time_delta_s)
    {
      continue;
    }
    const auto neighbor_odometry = odometry_by_id_.find(scan.scan_id);
    if (neighbor_odometry == odometry_by_id_.end()) {
      continue;
    }
    const Pose2d neighbor_to_center = relativePose(
      center_odometry->second.pose, neighbor_odometry->second.pose);
    for (std::size_t point = 0U; point < scan.points.size(); point += config_.point_stride) {
      const auto transformed = transformPoint(scan.points[point], neighbor_to_center);
      if (std::isfinite(transformed.x) && std::isfinite(transformed.y)) {
        result.points.push_back(transformed);
      }
    }
    result.contributing_scan_ids.push_back(scan.scan_id);
  }
  if (result.contributing_scan_ids.size() < config_.minimum_contributing_scans) {
    result.rejection_reason = "insufficient_contributing_scans";
  } else if (result.points.size() < config_.minimum_points) {
    result.rejection_reason = "insufficient_submap_points";
  } else {
    result.available = true;
    result.rejection_reason = "available";
  }
  return result;
}

}  // namespace embodied_slam
