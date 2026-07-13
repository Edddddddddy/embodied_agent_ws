#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct PoseGraphConstraint
{
  int source_id{0};
  int target_id{0};
  Pose2d relative_pose;
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct PoseGraphOptimizerConfig
{
  std::size_t max_iterations{50U};
  double relative_error_tolerance{1e-5};
  double huber_k{1.345};
  double minimum_covariance_eigenvalue{1e-8};
};

struct PoseGraphResult
{
  std::unordered_map<int, Pose2d> poses;
  double initial_error{0.0};
  double final_error{0.0};
  std::size_t iterations{0U};
};

class GtsamPoseGraphOptimizer
{
public:
  explicit GtsamPoseGraphOptimizer(PoseGraphOptimizerConfig config = {});

  PoseGraphResult optimize(
    const std::unordered_map<int, Pose2d> & initial_poses,
    const std::vector<PoseGraphConstraint> & constraints) const;

private:
  PoseGraphOptimizerConfig config_;
};

}  // namespace embodied_slam
