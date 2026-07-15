#include "embodied_slam/lidar_submap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace embodied_slam
{
namespace
{

void validateConfig(const LidarSubmapConfig & config)
{
  if (config.point_stride == 0U || config.minimum_contributing_scans == 0U ||
    config.minimum_points < 3U || !std::isfinite(config.maximum_time_delta_s) ||
    config.maximum_time_delta_s <= 0.0)
  {
    throw std::invalid_argument("invalid lidar submap configuration");
  }
}

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

LidarSubmapGeometry assembleLidarSubmap(
  const std::int64_t center_frame_id,
  const std::vector<LidarSubmapFrameView> & ordered_frames,
  const LidarSubmapConfig & config)
{
  validateConfig(config);
  LidarSubmapGeometry result;
  result.center_frame_id = center_frame_id;
  const auto center = std::find_if(
    ordered_frames.begin(), ordered_frames.end(),
    [center_frame_id](const auto & frame) {return frame.frame_id == center_frame_id;});
  if (center == ordered_frames.end() || center->points == nullptr ||
    !std::isfinite(center->stamp_s) || !std::isfinite(center->odometry_pose.x) ||
    !std::isfinite(center->odometry_pose.y) || !std::isfinite(center->odometry_pose.yaw))
  {
    result.rejection_reason = "missing_center_scan_or_odometry";
    return result;
  }

  for (const auto & frame : ordered_frames) {
    if (frame.points == nullptr || !std::isfinite(frame.stamp_s) ||
      std::abs(frame.stamp_s - center->stamp_s) > config.maximum_time_delta_s ||
      !std::isfinite(frame.odometry_pose.x) || !std::isfinite(frame.odometry_pose.y) ||
      !std::isfinite(frame.odometry_pose.yaw))
    {
      continue;
    }
    const Pose2d neighbor_to_center = relativePose(
      center->odometry_pose, frame.odometry_pose);
    for (std::size_t point = 0U; point < frame.points->size(); point += config.point_stride) {
      const auto transformed = transformPoint(frame.points->at(point), neighbor_to_center);
      if (std::isfinite(transformed.x) && std::isfinite(transformed.y)) {
        result.points.push_back(transformed);
      }
    }
    result.contributing_frame_ids.push_back(frame.frame_id);
  }
  if (result.contributing_frame_ids.size() < config.minimum_contributing_scans) {
    result.rejection_reason = "insufficient_contributing_scans";
  } else if (result.points.size() < config.minimum_points) {
    result.rejection_reason = "insufficient_submap_points";
  } else {
    result.available = true;
    result.rejection_reason = "available";
  }
  return result;
}

LidarSubmapBuilder::LidarSubmapBuilder(
  const std::vector<LidarScanRecord> & scans,
  const std::vector<LidarOdometryRecord> & odometry,
  LidarSubmapConfig config)
: scans_(scans), config_(config)
{
  validateConfig(config_);
  if (scans.empty() || odometry.empty()) {
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
  std::vector<LidarSubmapFrameView> frames;
  frames.reserve(last - first + 1U);
  for (std::size_t index = first; index <= last; ++index) {
    const auto & scan = scans_[index];
    const auto neighbor_odometry = odometry_by_id_.find(scan.scan_id);
    if (neighbor_odometry == odometry_by_id_.end()) {
      continue;
    }
    frames.push_back({
      scan.scan_id, scan.stamp_s, &scan.points, neighbor_odometry->second.pose});
  }
  const auto geometry = assembleLidarSubmap(center_scan_id, frames, config_);
  result.available = geometry.available;
  result.points = geometry.points;
  result.rejection_reason = geometry.rejection_reason;
  result.contributing_scan_ids.reserve(geometry.contributing_frame_ids.size());
  for (const auto id : geometry.contributing_frame_ids) {
    result.contributing_scan_ids.push_back(static_cast<int>(id));
  }
  return result;
}

}  // namespace embodied_slam
