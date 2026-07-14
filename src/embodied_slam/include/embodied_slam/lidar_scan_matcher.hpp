#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "embodied_slam/lidar_loop_descriptor.hpp"
#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct LidarScanMatcherConfig
{
  std::size_t point_stride{2U};
  std::size_t minimum_points{30U};
  std::size_t coarse_iterations{12U};
  std::size_t fine_iterations{20U};
  double coarse_max_correspondence_m{1.0};
  double fine_max_correspondence_m{0.25};
  double trim_fraction{0.80};
  double translation_tolerance_m{1e-4};
  double rotation_tolerance_rad{1e-4};
  double maximum_translation_m{2.0};
  double minimum_inlier_ratio{0.35};
  double maximum_rmse_m{0.18};
  double minimum_observability_ratio{0.005};
  double ambiguity_overlap_tolerance{0.03};
  double ambiguity_rmse_tolerance_m{0.02};
  double minimum_ambiguous_yaw_separation_rad{1.0};
  double maximum_prior_yaw_deviation_rad{0.8};
  // 长时间回访时里程计平移会累计漂移，默认只用偏航解除走廊半周对称。
  // 只有上游能保证局部平移先验可靠时，才显式打开平移一致性门限。
  bool gate_prior_translation{false};
  double maximum_prior_translation_deviation_m{3.0};
};

struct LidarScanMatchResult
{
  bool available{false};
  bool converged{false};
  bool accepted{false};
  bool yaw_ambiguous{false};
  bool odometry_prior_used{false};
  bool odometry_prior_consistent{false};
  Pose2d target_to_source;
  double descriptor_yaw_offset_rad{0.0};
  double selected_initial_yaw_rad{0.0};
  std::size_t iterations{0U};
  std::size_t source_points{0U};
  std::size_t target_points{0U};
  std::size_t correspondences{0U};
  double inlier_ratio{0.0};
  double bidirectional_overlap_ratio{0.0};
  double rmse_m{0.0};
  double observability_ratio{0.0};
  double alternate_overlap_ratio{0.0};
  double alternate_yaw_separation_rad{0.0};
  double odometry_prior_yaw_error_rad{0.0};
  double odometry_prior_translation_error_m{0.0};
  std::string rejection_reason;
};

// target_to_source 的语义与位姿图约束一致：把候选历史扫描(target)变换到
// 当前查询扫描(source)坐标系。描述子只提供偏航初值，ICP 不读取里程计或真值。
LidarScanMatchResult matchLidarScans(
  const std::vector<LidarPoint2D> & source,
  const std::vector<LidarPoint2D> & target,
  double descriptor_yaw_offset_rad,
  const LidarScanMatcherConfig & config = {},
  std::optional<Pose2d> odometry_prior = std::nullopt);

}  // namespace embodied_slam
