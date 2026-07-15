#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "embodied_navigation/obstacle_types.hpp"

namespace embodied_navigation
{

enum class AssociationStrategy
{
  GreedyNearest,
  GlobalNearest,
};

enum class AssociationMetric
{
  Euclidean,
  Mahalanobis,
};

struct AssociationPrediction
{
  Point2d position;
  // 仅保存二维位置协方差的对角项；当前感知输入没有 xy 互协方差，避免伪造精度。
  Point2d position_variance;
};

AssociationStrategy association_strategy_from_string(const std::string & value);
const char * to_string(AssociationStrategy strategy) noexcept;
AssociationMetric association_metric_from_string(const std::string & value);
const char * to_string(AssociationMetric metric) noexcept;

using ObservationAssignment = std::vector<std::optional<std::size_t>>;

// 两种策略共享同一门限语义，便于固定输入 A/B。GlobalNearest 用匈牙利算法
// 一次求解全部轨迹与观测，避免前面的轨迹贪心占走后续轨迹唯一可用的观测。
ObservationAssignment assign_gated_observations(
  const std::vector<Point2d> & predicted_positions,
  const std::vector<Point2d> & observations,
  double association_distance_m,
  AssociationStrategy strategy);

// Mahalanobis 模式使用 S = P_prediction + R_measurement 计算二维 NIS。
// 硬欧氏距离门仍然保留，防止协方差异常膨胀后把远处观测吸进旧轨迹。
ObservationAssignment assign_gated_observations(
  const std::vector<AssociationPrediction> & predictions,
  const std::vector<Point2d> & observations,
  double measurement_noise_variance,
  double association_distance_m,
  double association_nis_gate,
  AssociationStrategy strategy,
  AssociationMetric metric);

}  // namespace embodied_navigation
