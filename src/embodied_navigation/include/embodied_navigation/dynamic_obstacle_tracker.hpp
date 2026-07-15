#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "embodied_navigation/gated_observation_assignment.hpp"
#include "embodied_navigation/obstacle_types.hpp"

namespace embodied_navigation
{

struct TrackedObstacle
{
  std::string id;
  Point2d position;
  Point2d velocity;
  double radius{0.25};
  double confidence{0.5};
  double last_seen_s{0.0};
  std::size_t observation_count{0U};
};

enum class MotionModel
{
  CurrentOnly,
  ConstantVelocity,
  Kalman,
  Imm,
};

MotionModel motion_model_from_string(const std::string & value);
const char * to_string(MotionModel model) noexcept;

struct TrackerConfig
{
  double association_distance_m{0.8};
  double velocity_smoothing{0.65};
  double track_timeout_s{1.0};
  double default_radius_m{0.25};
  AssociationStrategy association_strategy{AssociationStrategy::GlobalNearest};
  // 纯库默认保留欧氏语义；ROS 运行时参数默认启用 Mahalanobis，兼容旧的直接调用方。
  AssociationMetric association_metric{AssociationMetric::Euclidean};
  // 9.210 是二维卡方分布 99% 分位点；低于它的创新才进入全局分配。
  double association_nis_gate{9.210};
  MotionModel motion_model{MotionModel::ConstantVelocity};
  double measurement_noise_variance{0.01};
  double process_noise_variance{0.2};
  double imm_stationary_process_noise{0.01};
  double imm_maneuver_process_noise{1.0};
  double imm_stay_probability{0.94};
  double imm_stationary_velocity_decay{0.2};
};

class DynamicObstacleTracker
{
public:
  explicit DynamicObstacleTracker(TrackerConfig config = {});
  ~DynamicObstacleTracker();
  DynamicObstacleTracker(DynamicObstacleTracker &&) noexcept;
  DynamicObstacleTracker & operator=(DynamicObstacleTracker &&) noexcept;
  DynamicObstacleTracker(const DynamicObstacleTracker &) = delete;
  DynamicObstacleTracker & operator=(const DynamicObstacleTracker &) = delete;

  const std::vector<TrackedObstacle> & update(
    const std::vector<Point2d> & observations, double timestamp_s);
  const std::vector<TrackedObstacle> & tracks() const;
  void clear();

private:
  // 运动模型及其协方差全部隐藏在实现中，ROS 节点只依赖稳定的 update seam。
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace embodied_navigation
