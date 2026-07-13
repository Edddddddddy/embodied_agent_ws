#include "embodied_slam/gtsam_pose_graph.hpp"

#include <algorithm>
#include <utility>

#include <Eigen/Eigenvalues>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace embodied_slam
{

namespace
{
gtsam::Pose2 to_gtsam(const Pose2d & pose)
{
  return {pose.x, pose.y, pose.yaw};
}

Pose2d from_gtsam(const gtsam::Pose2 & pose)
{
  return {pose.x(), pose.y(), pose.theta()};
}

Eigen::Matrix3d make_positive_definite(Eigen::Matrix3d covariance, double minimum_eigenvalue)
{
  // Scan matching 在退化几何中可能给出接近奇异的协方差。直接求逆会使后端崩溃，
  // 因此先对称化，再把极小或负特征值抬到可配置下限。
  covariance = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix3d::Identity() * minimum_eigenvalue;
  }
  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  eigenvalues = eigenvalues.array().max(minimum_eigenvalue);
  return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}
}  // namespace

GtsamPoseGraphOptimizer::GtsamPoseGraphOptimizer(PoseGraphOptimizerConfig config)
: config_(std::move(config))
{
}

PoseGraphResult GtsamPoseGraphOptimizer::optimize(
  const std::unordered_map<int, Pose2d> & initial_poses,
  const std::vector<PoseGraphConstraint> & constraints) const
{
  if (initial_poses.empty()) {
    return {};
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  int anchor_id = initial_poses.begin()->first;
  for (const auto & [id, pose] : initial_poses) {
    anchor_id = std::min(anchor_id, id);
    initial.insert(static_cast<gtsam::Key>(id), to_gtsam(pose));
  }

  // 固定最早一帧消除 SE(2) 规范自由度，否则整张图可以整体平移和旋转而没有唯一解。
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(
    static_cast<gtsam::Key>(anchor_id), to_gtsam(initial_poses.at(anchor_id)),
    gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished()));

  for (const auto & constraint : constraints) {
    if (!initial_poses.count(constraint.source_id) || !initial_poses.count(constraint.target_id)) {
      continue;
    }
    const Eigen::Matrix3d covariance = make_positive_definite(
      constraint.covariance, config_.minimum_covariance_eigenvalue);
    auto gaussian = gtsam::noiseModel::Gaussian::Covariance(covariance);
    gtsam::SharedNoiseModel noise = gaussian;
    if (config_.huber_k > 0.0) {
      noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_k), gaussian);
    }
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      static_cast<gtsam::Key>(constraint.source_id),
      static_cast<gtsam::Key>(constraint.target_id),
      to_gtsam(constraint.relative_pose), noise);
  }

  gtsam::LevenbergMarquardtParams parameters;
  parameters.maxIterations = config_.max_iterations;
  parameters.relativeErrorTol = config_.relative_error_tolerance;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  PoseGraphResult output;
  output.initial_error = graph.error(initial);
  const gtsam::Values result = optimizer.optimize();
  output.final_error = graph.error(result);
  output.iterations = optimizer.iterations();
  for (const auto & item : initial_poses) {
    const int id = item.first;
    output.poses.emplace(
      id, from_gtsam(result.at<gtsam::Pose2>(static_cast<gtsam::Key>(id))));
  }
  return output;
}

}  // namespace embodied_slam
