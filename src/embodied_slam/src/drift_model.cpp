#include "embodied_slam/drift_model.hpp"

#include <cmath>
#include <utility>

namespace embodied_slam
{

DriftModel::DriftModel(DriftConfig config)
: config_(std::move(config)), random_engine_(config_.random_seed)
{
}

Pose2d DriftModel::update(const Pose2d & reference_pose)
{
  if (!initialized_) {
    // 两条轨迹从同一个原点开始，后续误差只来自注入项，不混入初始坐标偏差。
    previous_reference_ = reference_pose;
    estimate_ = reference_pose;
    initialized_ = true;
    return estimate_;
  }

  const double world_dx = reference_pose.x - previous_reference_.x;
  const double world_dy = reference_pose.y - previous_reference_.y;
  const double cos_yaw = std::cos(previous_reference_.yaw);
  const double sin_yaw = std::sin(previous_reference_.yaw);
  const double forward = cos_yaw * world_dx + sin_yaw * world_dy;
  const double lateral = -sin_yaw * world_dx + cos_yaw * world_dy;
  const double traveled = std::hypot(forward, lateral);

  const double noisy_forward =
    config_.linear_scale * forward +
    config_.translation_noise_stddev * unit_normal_(random_engine_);
  const double noisy_lateral =
    config_.lateral_scale * lateral +
    config_.translation_noise_stddev * unit_normal_(random_engine_);
  const double delta_yaw = normalize_angle(reference_pose.yaw - previous_reference_.yaw);
  const double noisy_delta_yaw =
    delta_yaw + config_.yaw_bias_per_meter * traveled +
    config_.yaw_noise_stddev * unit_normal_(random_engine_);

  const double estimate_cos = std::cos(estimate_.yaw);
  const double estimate_sin = std::sin(estimate_.yaw);
  estimate_.x += estimate_cos * noisy_forward - estimate_sin * noisy_lateral;
  estimate_.y += estimate_sin * noisy_forward + estimate_cos * noisy_lateral;
  estimate_.yaw = normalize_angle(estimate_.yaw + noisy_delta_yaw);
  previous_reference_ = reference_pose;
  return estimate_;
}

void DriftModel::reset()
{
  initialized_ = false;
  previous_reference_ = {};
  estimate_ = {};
  random_engine_.seed(config_.random_seed);
  // normal_distribution 可能缓存 Box-Muller 生成的第二个样本；只重置随机引擎会让
  // 同一个 seed 在重跑基准时得到不同轨迹，因此分布状态也必须一起清空。
  unit_normal_.reset();
}

bool DriftModel::initialized() const
{
  return initialized_;
}

const Pose2d & DriftModel::estimate() const
{
  return estimate_;
}

}  // namespace embodied_slam
