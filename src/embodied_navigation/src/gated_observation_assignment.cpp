#include "embodied_navigation/gated_observation_assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace embodied_navigation
{
namespace
{

double squared_distance(const Point2d & left, const Point2d & right)
{
  const double x = left.x - right.x;
  const double y = left.y - right.y;
  return x * x + y * y;
}

ObservationAssignment greedy_assignment(
  const std::vector<Point2d> & predictions,
  const std::vector<Point2d> & observations,
  const double maximum_squared_distance)
{
  ObservationAssignment assignment(predictions.size());
  std::vector<bool> used(observations.size(), false);
  for (std::size_t row = 0U; row < predictions.size(); ++row) {
    double best_cost = maximum_squared_distance;
    for (std::size_t column = 0U; column < observations.size(); ++column) {
      if (used[column]) {
        continue;
      }
      const double cost = squared_distance(predictions[row], observations[column]);
      if (cost < best_cost) {
        best_cost = cost;
        assignment[row] = column;
      }
    }
    if (assignment[row]) {
      used[*assignment[row]] = true;
    }
  }
  return assignment;
}

ObservationAssignment global_assignment(
  const std::vector<Point2d> & predictions,
  const std::vector<Point2d> & observations,
  const double maximum_squared_distance)
{
  const std::size_t rows = predictions.size();
  if (rows == 0U) {
    return {};
  }
  // 每条轨迹拥有一个私有 dummy 列，代表“本帧未匹配”。列数因此始终不少于行数，
  // 既能处理遮挡，也避免为了方阵补零而把禁配边误认为免费匹配。
  const std::size_t real_columns = observations.size();
  const std::size_t columns = real_columns + rows;
  const double forbidden_cost = std::max(
    1.0, maximum_squared_distance * static_cast<double>(rows + columns + 4U));
  const auto cost = [&](const std::size_t row, const std::size_t column) {
      if (column < real_columns) {
        const double distance = squared_distance(predictions[row], observations[column]);
        return distance < maximum_squared_distance ? distance : forbidden_cost;
      }
      return column - real_columns == row ? maximum_squared_distance : forbidden_cost;
    };

  // 矩形匈牙利算法，1-based 数组保持增广路径公式清晰；复杂度 O(n^2 m)，
  // 对机器人现场常见的少量动态目标远低于感知与 costmap 开销。
  std::vector<double> row_potential(rows + 1U, 0.0);
  std::vector<double> column_potential(columns + 1U, 0.0);
  std::vector<std::size_t> matched_row(columns + 1U, 0U);
  std::vector<std::size_t> previous_column(columns + 1U, 0U);
  for (std::size_t row = 1U; row <= rows; ++row) {
    matched_row[0] = row;
    std::size_t current_column = 0U;
    std::vector<double> minimum_slack(
      columns + 1U, std::numeric_limits<double>::infinity());
    std::vector<bool> visited(columns + 1U, false);
    do {
      visited[current_column] = true;
      const std::size_t current_row = matched_row[current_column];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t next_column = 0U;
      for (std::size_t column = 1U; column <= columns; ++column) {
        if (visited[column]) {
          continue;
        }
        const double reduced_cost = cost(current_row - 1U, column - 1U) -
          row_potential[current_row] - column_potential[column];
        if (reduced_cost < minimum_slack[column]) {
          minimum_slack[column] = reduced_cost;
          previous_column[column] = current_column;
        }
        if (minimum_slack[column] < delta) {
          delta = minimum_slack[column];
          next_column = column;
        }
      }
      for (std::size_t column = 0U; column <= columns; ++column) {
        if (visited[column]) {
          row_potential[matched_row[column]] += delta;
          column_potential[column] -= delta;
        } else {
          minimum_slack[column] -= delta;
        }
      }
      current_column = next_column;
    } while (matched_row[current_column] != 0U);

    do {
      const std::size_t predecessor = previous_column[current_column];
      matched_row[current_column] = matched_row[predecessor];
      current_column = predecessor;
    } while (current_column != 0U);
  }

  ObservationAssignment assignment(rows);
  for (std::size_t column = 1U; column <= real_columns; ++column) {
    if (matched_row[column] == 0U) {
      continue;
    }
    const std::size_t row = matched_row[column] - 1U;
    if (cost(row, column - 1U) < maximum_squared_distance) {
      assignment[row] = column - 1U;
    }
  }
  return assignment;
}

}  // namespace

AssociationStrategy association_strategy_from_string(const std::string & value)
{
  if (value == "greedy_nearest" || value == "greedy") {
    return AssociationStrategy::GreedyNearest;
  }
  if (value == "global_nearest" || value == "global") {
    return AssociationStrategy::GlobalNearest;
  }
  throw std::invalid_argument(
          "association_strategy must be one of: greedy_nearest, global_nearest");
}

const char * to_string(const AssociationStrategy strategy) noexcept
{
  return strategy == AssociationStrategy::GlobalNearest ? "global_nearest" : "greedy_nearest";
}

ObservationAssignment assign_gated_observations(
  const std::vector<Point2d> & predicted_positions,
  const std::vector<Point2d> & observations,
  const double association_distance_m,
  const AssociationStrategy strategy)
{
  if (!std::isfinite(association_distance_m) || association_distance_m <= 0.0) {
    throw std::invalid_argument("association distance must be finite and positive");
  }
  const auto finite = [](const Point2d & point) {
      return std::isfinite(point.x) && std::isfinite(point.y);
    };
  if (!std::all_of(predicted_positions.begin(), predicted_positions.end(), finite) ||
    !std::all_of(observations.begin(), observations.end(), finite))
  {
    throw std::invalid_argument("association points must be finite");
  }
  const double maximum_squared_distance = association_distance_m * association_distance_m;
  return strategy == AssociationStrategy::GlobalNearest ?
         global_assignment(predicted_positions, observations, maximum_squared_distance) :
         greedy_assignment(predicted_positions, observations, maximum_squared_distance);
}

}  // namespace embodied_navigation
