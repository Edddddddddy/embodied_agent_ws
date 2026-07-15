#include "embodied_slam/gtsam_pose_graph.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Eigenvalues>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

namespace embodied_slam
{

RobustKernel robust_kernel_from_string(const std::string & value)
{
  std::string normalized = value;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (normalized == "none") {
    return RobustKernel::kNone;
  }
  if (normalized == "huber") {
    return RobustKernel::kHuber;
  }
  if (normalized == "cauchy") {
    return RobustKernel::kCauchy;
  }
  throw std::invalid_argument("unsupported robust kernel: " + value);
}

const char * robust_kernel_name(RobustKernel value)
{
  switch (value) {
    case RobustKernel::kNone:
      return "none";
    case RobustKernel::kHuber:
      return "huber";
    case RobustKernel::kCauchy:
      return "cauchy";
  }
  return "unknown";
}

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

gtsam::SharedNoiseModel robust_noise(
  RobustKernel kernel, double parameter, const gtsam::SharedNoiseModel & gaussian)
{
  if (kernel == RobustKernel::kHuber) {
    return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(parameter), gaussian);
  }
  if (kernel == RobustKernel::kCauchy) {
    return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(parameter), gaussian);
  }
  return gaussian;
}

Pose2d relative_pose(const Pose2d & source, const Pose2d & target)
{
  const double dx = target.x - source.x;
  const double dy = target.y - source.y;
  const double cosine = std::cos(source.yaw);
  const double sine = std::sin(source.yaw);
  return {
    cosine * dx + sine * dy,
    -sine * dx + cosine * dy,
    std::atan2(std::sin(target.yaw - source.yaw), std::cos(target.yaw - source.yaw))};
}

struct ConstraintResidual
{
  double translation_m{0.0};
  double yaw_rad{0.0};
};

ConstraintResidual constraint_residual(
  const PoseGraphConstraint & constraint, const std::unordered_map<int, Pose2d> & initial_poses)
{
  const Pose2d predicted = relative_pose(
    initial_poses.at(constraint.source_id), initial_poses.at(constraint.target_id));
  const double translation_residual = std::hypot(
    constraint.relative_pose.x - predicted.x, constraint.relative_pose.y - predicted.y);
  const double yaw_delta = constraint.relative_pose.yaw - predicted.yaw;
  const double yaw_residual = std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta)));
  return {translation_residual, yaw_residual};
}

class SwitchableBetweenFactor final
  : public gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2, double>
{
public:
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose2, gtsam::Pose2, double>;

  SwitchableBetweenFactor(
    gtsam::Key source_key, gtsam::Key target_key, gtsam::Key switch_key,
    gtsam::Pose2 measured, const gtsam::SharedNoiseModel & noise)
  : Base(noise, source_key, target_key, switch_key), measured_(std::move(measured))
  {
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new SwitchableBetweenFactor(*this)));
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose2 & source, const gtsam::Pose2 & target, const double & switch_value,
    boost::optional<gtsam::Matrix &> source_jacobian = boost::none,
    boost::optional<gtsam::Matrix &> target_jacobian = boost::none,
    boost::optional<gtsam::Matrix &> switch_jacobian = boost::none) const override
  {
    gtsam::Matrix source_between_jacobian;
    gtsam::Matrix target_between_jacobian;
    const gtsam::Pose2 predicted = source.between(
      target,
      source_jacobian ? boost::optional<gtsam::Matrix &>(source_between_jacobian) : boost::none,
      target_jacobian ? boost::optional<gtsam::Matrix &>(target_between_jacobian) : boost::none);
    gtsam::Matrix prediction_jacobian;
    const gtsam::Vector residual = measured_.localCoordinates(
      predicted, boost::none,
      (source_jacobian || target_jacobian) ?
      boost::optional<gtsam::Matrix &>(prediction_jacobian) : boost::none);

    if (source_jacobian) {
      *source_jacobian = switch_value * prediction_jacobian * source_between_jacobian;
    }
    if (target_jacobian) {
      *target_jacobian = switch_value * prediction_jacobian * target_between_jacobian;
    }
    if (switch_jacobian) {
      *switch_jacobian = residual;
    }
    return switch_value * residual;
  }

private:
  gtsam::Pose2 measured_;
};
}  // namespace

GtsamPoseGraphOptimizer::GtsamPoseGraphOptimizer(PoseGraphOptimizerConfig config)
: config_(std::move(config))
{
  if (config_.enable_nonlocal_consistency_gate &&
    (!(config_.max_nonlocal_translation_residual_m > 0.0) ||
    !(config_.max_nonlocal_yaw_residual_rad > 0.0) ||
    !std::isfinite(config_.max_nonlocal_translation_residual_m) ||
    !std::isfinite(config_.max_nonlocal_yaw_residual_rad)))
  {
    throw std::invalid_argument("consistency-gate residual limits must be finite and positive");
  }
  if (config_.enable_scan_overlap_gate &&
    ((!std::isfinite(config_.minimum_scan_overlap_ratio)) ||
    config_.minimum_scan_overlap_ratio < 0.0 || config_.minimum_scan_overlap_ratio > 1.0 ||
    (!std::isfinite(config_.scan_overlap_gate_min_translation_residual_m)) ||
    config_.scan_overlap_gate_min_translation_residual_m < 0.0))
  {
    throw std::invalid_argument(
            "scan-overlap gate thresholds must be finite, with overlap in [0, 1]");
  }
  if (config_.enable_switchable_loop_constraints &&
    ((!std::isfinite(config_.switch_prior_sigma)) || !(config_.switch_prior_sigma > 0.0) ||
    (!std::isfinite(config_.switch_suppression_threshold)) ||
    config_.switch_suppression_threshold < 0.0 ||
    config_.switch_suppression_threshold > 1.0))
  {
    throw std::invalid_argument(
            "switchable-constraint prior sigma must be positive and threshold in [0, 1]");
  }
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
  PoseGraphResult output;
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

  std::vector<std::pair<gtsam::Key, SwitchableConstraintEstimate>> switch_variables;
  std::size_t switch_index = 0U;
  for (const auto & constraint : constraints) {
    if (!initial_poses.count(constraint.source_id) || !initial_poses.count(constraint.target_id)) {
      continue;
    }
    const Eigen::Matrix3d covariance = make_positive_definite(
      constraint.covariance, config_.minimum_covariance_eigenvalue);
    auto gaussian = gtsam::noiseModel::Gaussian::Covariance(covariance);
    gtsam::SharedNoiseModel noise = gaussian;
    const auto id_separation = static_cast<std::size_t>(
      std::abs(constraint.source_id - constraint.target_id));
    const bool is_loop = id_separation >= config_.loop_constraint_min_id_separation;
    if (is_loop &&
      (config_.enable_nonlocal_consistency_gate || config_.enable_scan_overlap_gate))
    {
      const auto residual = constraint_residual(constraint, initial_poses);
      if (config_.enable_nonlocal_consistency_gate &&
        (residual.translation_m > config_.max_nonlocal_translation_residual_m ||
        residual.yaw_rad > config_.max_nonlocal_yaw_residual_rad))
      {
        // 不使用真值，只比较候选边与入图前位姿估计形成的闭环残差。硬门控只处理明显离群边，
        // 正常残差仍交给鲁棒核连续降权，避免把少量累计漂移误当作错误回环。
        ++output.consistency_rejected_constraints;
        continue;
      }
      if (config_.enable_scan_overlap_gate) {
        if (!constraint.scan_overlap_ratio.has_value()) {
          // 扫描缺失或有效点不足时 fail-open，不能把“无证据”伪装成“不重合”。
          ++output.scan_overlap_unavailable_constraints;
        } else {
          ++output.scan_overlap_evaluated_constraints;
          if (residual.translation_m >
            config_.scan_overlap_gate_min_translation_residual_m &&
            *constraint.scan_overlap_ratio < config_.minimum_scan_overlap_ratio)
          {
            // 单帧低重合不足以否定 Karto 的链式子图匹配；只有图创新也可疑时才使用第二证据拒绝。
            ++output.scan_overlap_rejected_constraints;
            continue;
          }
        }
      }
    }
    const bool should_robustify =
      config_.robust_kernel != RobustKernel::kNone && config_.robust_kernel_k > 0.0 &&
      (!config_.robustify_loop_constraints_only || is_loop);
    if (should_robustify) {
      // 局部里程计链通常连续可靠；loop-only 模式仅抑制非局部闭环的离群残差，
      // 避免错误回环把整条走廊轨迹强行拉回，同时不削弱正常相邻边。
      noise = robust_noise(config_.robust_kernel, config_.robust_kernel_k, gaussian);
      ++output.robustified_constraints;
    }
    if (is_loop && config_.enable_switchable_loop_constraints) {
      const gtsam::Key switch_key = gtsam::Symbol('s', switch_index++);
      initial.insert(switch_key, 1.0);
      // 每条非局部边拥有独立 switch：正确回环受 s=1 先验保护，残差很大的错误边会把
      // 自己的 s 压向 0，而不是把整张位姿图拉坏。它仍然不能替代前端地点识别证据。
      graph.emplace_shared<SwitchableBetweenFactor>(
        static_cast<gtsam::Key>(constraint.source_id),
        static_cast<gtsam::Key>(constraint.target_id), switch_key,
        to_gtsam(constraint.relative_pose), noise);
      graph.emplace_shared<gtsam::PriorFactor<double>>(
        switch_key, 1.0,
        gtsam::noiseModel::Isotropic::Sigma(1U, config_.switch_prior_sigma));
      switch_variables.push_back(
        {switch_key, {constraint.source_id, constraint.target_id, 1.0}});
      ++output.switchable_constraints;
    } else {
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
        static_cast<gtsam::Key>(constraint.source_id),
        static_cast<gtsam::Key>(constraint.target_id),
        to_gtsam(constraint.relative_pose), noise);
    }
    ++output.constraints_used;
  }

  gtsam::LevenbergMarquardtParams parameters;
  parameters.maxIterations = config_.max_iterations;
  parameters.relativeErrorTol = config_.relative_error_tolerance;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  output.initial_error = graph.error(initial);
  const gtsam::Values result = optimizer.optimize();
  output.final_error = graph.error(result);
  output.iterations = optimizer.iterations();
  double switch_sum = 0.0;
  for (const auto & [key, metadata] : switch_variables) {
    SwitchableConstraintEstimate estimate = metadata;
    estimate.value = result.at<double>(key);
    output.minimum_switch_value = std::min(output.minimum_switch_value, estimate.value);
    switch_sum += estimate.value;
    if (estimate.value < config_.switch_suppression_threshold) {
      ++output.switch_suppressed_constraints;
    }
    output.switch_estimates.push_back(estimate);
  }
  if (!switch_variables.empty()) {
    output.mean_switch_value = switch_sum / static_cast<double>(switch_variables.size());
  }
  for (const auto & item : initial_poses) {
    const int id = item.first;
    output.poses.emplace(
      id, from_gtsam(result.at<gtsam::Pose2>(static_cast<gtsam::Key>(id))));
  }
  return output;
}

}  // namespace embodied_slam
