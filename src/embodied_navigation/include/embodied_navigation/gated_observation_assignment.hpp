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

AssociationStrategy association_strategy_from_string(const std::string & value);
const char * to_string(AssociationStrategy strategy) noexcept;

using ObservationAssignment = std::vector<std::optional<std::size_t>>;

// 两种策略共享同一门限语义，便于固定输入 A/B。GlobalNearest 用匈牙利算法
// 一次求解全部轨迹与观测，避免前面的轨迹贪心占走后续轨迹唯一可用的观测。
ObservationAssignment assign_gated_observations(
  const std::vector<Point2d> & predicted_positions,
  const std::vector<Point2d> & observations,
  double association_distance_m,
  AssociationStrategy strategy);

}  // namespace embodied_navigation
