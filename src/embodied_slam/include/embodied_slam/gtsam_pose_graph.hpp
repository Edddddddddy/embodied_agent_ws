#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

enum class RobustKernel
{
  kNone,
  kHuber,
  kCauchy,
};

RobustKernel robust_kernel_from_string(const std::string & value);
const char * robust_kernel_name(RobustKernel value);

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
  RobustKernel robust_kernel{RobustKernel::kHuber};
  double robust_kernel_k{1.345};
  bool robustify_loop_constraints_only{false};
  std::size_t loop_constraint_min_id_separation{20U};
  double minimum_covariance_eigenvalue{1e-8};
};

struct PoseGraphResult
{
  std::unordered_map<int, Pose2d> poses;
  double initial_error{0.0};
  double final_error{0.0};
  std::size_t iterations{0U};
  std::size_t constraints_used{0U};
  std::size_t robustified_constraints{0U};
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
