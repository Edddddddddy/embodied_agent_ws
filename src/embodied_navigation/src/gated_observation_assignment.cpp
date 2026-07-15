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

constexpr double kMinimumVariance = 1e-9;

double squared_distance(const Point2d & left, const Point2d & right)
{
  const double x = left.x - right.x;
  const double y = left.y - right.y;
  return x * x + y * y;
}

struct AssociationCosts
{
  std::size_t rows{0U};
  std::size_t real_columns{0U};
  double unmatched_cost{0.0};
  double forbidden_cost{0.0};
  std::vector<double> values;

  double at(const std::size_t row, const std::size_t column) const
  {
    return values[row * real_columns + column];
  }

  bool feasible(const std::size_t row, const std::size_t column) const
  {
    return at(row, column) < forbidden_cost;
  }
};

AssociationCosts build_costs(
  const std::vector<AssociationPrediction> & predictions,
  const std::vector<Point2d> & observations,
  const double measurement_noise_variance,
  const double maximum_squared_distance,
  const double association_nis_gate,
  const AssociationMetric metric)
{
  AssociationCosts costs;
  costs.rows = predictions.size();
  costs.real_columns = observations.size();
  costs.unmatched_cost = metric == AssociationMetric::Mahalanobis ?
    association_nis_gate : maximum_squared_distance;
  const std::size_t columns = costs.real_columns + costs.rows;
  costs.forbidden_cost = std::max(
    1.0, costs.unmatched_cost * static_cast<double>(costs.rows + columns + 4U));
  costs.values.assign(costs.rows * costs.real_columns, costs.forbidden_cost);

  for (std::size_t row = 0U; row < costs.rows; ++row) {
    for (std::size_t column = 0U; column < costs.real_columns; ++column) {
      const double distance = squared_distance(
        predictions[row].position, observations[column]);
      if (distance >= maximum_squared_distance) {
        continue;
      }
      double cost = distance;
      if (metric == AssociationMetric::Mahalanobis) {
        const double variance_x = std::max(
          predictions[row].position_variance.x + measurement_noise_variance,
          kMinimumVariance);
        const double variance_y = std::max(
          predictions[row].position_variance.y + measurement_noise_variance,
          kMinimumVariance);
        const double delta_x = observations[column].x - predictions[row].position.x;
        const double delta_y = observations[column].y - predictions[row].position.y;
        cost = delta_x * delta_x / variance_x + delta_y * delta_y / variance_y;
        if (cost >= association_nis_gate) {
          continue;
        }
      }
      costs.values[row * costs.real_columns + column] = cost;
    }
  }
  return costs;
}

ObservationAssignment greedy_assignment(const AssociationCosts & costs)
{
  ObservationAssignment assignment(costs.rows);
  std::vector<bool> used(costs.real_columns, false);
  for (std::size_t row = 0U; row < costs.rows; ++row) {
    double best_cost = costs.unmatched_cost;
    for (std::size_t column = 0U; column < costs.real_columns; ++column) {
      if (used[column]) {
        continue;
      }
      const double candidate_cost = costs.at(row, column);
      if (candidate_cost < best_cost) {
        best_cost = candidate_cost;
        assignment[row] = column;
      }
    }
    if (assignment[row]) {
      used[*assignment[row]] = true;
    }
  }
  return assignment;
}

ObservationAssignment global_assignment(const AssociationCosts & costs)
{
  const std::size_t rows = costs.rows;
  if (rows == 0U) {
    return {};
  }
  // 每条轨迹拥有一个私有 dummy 列，代表“本帧未匹配”。列数因此始终不少于行数，
  // 既能处理遮挡，也避免为了方阵补零而把禁配边误认为免费匹配。
  const std::size_t real_columns = costs.real_columns;
  const std::size_t columns = real_columns + rows;
  const auto cost = [&](const std::size_t row, const std::size_t column) {
      if (column < real_columns) {
        return costs.at(row, column);
      }
      return column - real_columns == row ? costs.unmatched_cost : costs.forbidden_cost;
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
    if (costs.feasible(row, column - 1U)) {
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

AssociationMetric association_metric_from_string(const std::string & value)
{
  if (value == "euclidean") {
    return AssociationMetric::Euclidean;
  }
  if (value == "mahalanobis" || value == "nis") {
    return AssociationMetric::Mahalanobis;
  }
  throw std::invalid_argument("association_metric must be one of: euclidean, mahalanobis");
}

const char * to_string(const AssociationMetric metric) noexcept
{
  return metric == AssociationMetric::Mahalanobis ? "mahalanobis" : "euclidean";
}

ObservationAssignment assign_gated_observations(
  const std::vector<Point2d> & predicted_positions,
  const std::vector<Point2d> & observations,
  const double association_distance_m,
  const AssociationStrategy strategy)
{
  std::vector<AssociationPrediction> predictions;
  predictions.reserve(predicted_positions.size());
  for (const auto & position : predicted_positions) {
    predictions.push_back({position, {0.0, 0.0}});
  }
  return assign_gated_observations(
    predictions, observations, 0.0, association_distance_m, 1.0,
    strategy, AssociationMetric::Euclidean);
}

ObservationAssignment assign_gated_observations(
  const std::vector<AssociationPrediction> & predictions,
  const std::vector<Point2d> & observations,
  const double measurement_noise_variance,
  const double association_distance_m,
  const double association_nis_gate,
  const AssociationStrategy strategy,
  const AssociationMetric metric)
{
  if (!std::isfinite(association_distance_m) || association_distance_m <= 0.0) {
    throw std::invalid_argument("association distance must be finite and positive");
  }
  if (!std::isfinite(measurement_noise_variance) || measurement_noise_variance < 0.0) {
    throw std::invalid_argument("measurement noise variance must be finite and non-negative");
  }
  if (!std::isfinite(association_nis_gate) || association_nis_gate <= 0.0) {
    throw std::invalid_argument("association NIS gate must be finite and positive");
  }
  const auto finite = [](const Point2d & point) {
      return std::isfinite(point.x) && std::isfinite(point.y);
    };
  const auto valid_prediction = [&finite](const AssociationPrediction & prediction) {
      return finite(prediction.position) && finite(prediction.position_variance) &&
             prediction.position_variance.x >= 0.0 && prediction.position_variance.y >= 0.0;
    };
  if (!std::all_of(predictions.begin(), predictions.end(), valid_prediction) ||
    !std::all_of(observations.begin(), observations.end(), finite))
  {
    throw std::invalid_argument("association positions and variances must be finite and valid");
  }
  const double maximum_squared_distance = association_distance_m * association_distance_m;
  const auto costs = build_costs(
    predictions, observations, measurement_noise_variance, maximum_squared_distance,
    association_nis_gate, metric);
  return strategy == AssociationStrategy::GlobalNearest ?
         global_assignment(costs) : greedy_assignment(costs);
}

}  // namespace embodied_navigation
