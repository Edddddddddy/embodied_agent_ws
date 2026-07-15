#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "embodied_navigation/gated_observation_assignment.hpp"

namespace embodied_navigation
{
namespace
{

constexpr double kMeasurementVariance = 0.0025;
constexpr double kDistanceGateM = 1.0;
constexpr double kNisGate = 9.210;
constexpr std::size_t kTimingIterations = 10000U;

struct MetricResult
{
  AssociationMetric metric{AssociationMetric::Euclidean};
  std::size_t correct_identity_matches{0U};
  std::size_t unmatched_tracks{0U};
  double identity_position_rmse_m{0.0};
  double mean_assignment_time_us{0.0};
};

const std::vector<AssociationPrediction> & predictions()
{
  static const std::vector<AssociationPrediction> value{
    {{0.0, 0.0}, {0.0025, 0.0025}},
    {{0.4, 0.0}, {0.25, 0.25}}};
  return value;
}

const std::vector<Point2d> & observations()
{
  static const std::vector<Point2d> value{{-0.2, 0.0}, {0.1, 0.0}};
  return value;
}

ObservationAssignment assign(const AssociationMetric metric)
{
  return assign_gated_observations(
    predictions(), observations(), kMeasurementVariance, kDistanceGateM, kNisGate,
    AssociationStrategy::GlobalNearest, metric);
}

MetricResult run_metric(const AssociationMetric metric)
{
  const auto started = std::chrono::steady_clock::now();
  ObservationAssignment assignment;
  for (std::size_t iteration = 0U; iteration < kTimingIterations; ++iteration) {
    assignment = assign(metric);
  }
  const auto finished = std::chrono::steady_clock::now();

  // 真值身份为 track_1 -> observation[1]、track_2 -> observation[0]。
  const std::vector<std::size_t> expected_observations{1U, 0U};
  MetricResult result;
  result.metric = metric;
  double squared_error = 0.0;
  for (std::size_t track = 0U; track < expected_observations.size(); ++track) {
    if (!assignment[track]) {
      ++result.unmatched_tracks;
      squared_error += kDistanceGateM * kDistanceGateM;
      continue;
    }
    result.correct_identity_matches +=
      *assignment[track] == expected_observations[track] ? 1U : 0U;
    const auto & actual = observations()[*assignment[track]];
    const auto & expected = observations()[expected_observations[track]];
    const double delta_x = actual.x - expected.x;
    const double delta_y = actual.y - expected.y;
    squared_error += delta_x * delta_x + delta_y * delta_y;
  }
  result.identity_position_rmse_m = std::sqrt(
    squared_error / static_cast<double>(expected_observations.size()));
  result.mean_assignment_time_us =
    std::chrono::duration<double, std::micro>(finished - started).count() /
    static_cast<double>(kTimingIterations);
  return result;
}

void write_report(std::ostream & output, const std::vector<MetricResult> & results)
{
  output << std::fixed << std::setprecision(8)
         << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"scenario\": \"heteroscedastic_crossing_v1\",\n"
         << "  \"association_strategy\": \"global_nearest\",\n"
         << "  \"association_distance_m\": " << kDistanceGateM << ",\n"
         << "  \"association_nis_gate\": " << kNisGate << ",\n"
         << "  \"measurement_noise_variance\": " << kMeasurementVariance << ",\n"
         << "  \"prediction_position_variances\": [0.0025, 0.25],\n"
         << "  \"metrics\": [\n";
  for (std::size_t index = 0U; index < results.size(); ++index) {
    const auto & result = results[index];
    output << "    {\n"
           << "      \"metric\": \"" << to_string(result.metric) << "\",\n"
           << "      \"correct_identity_matches\": "
           << result.correct_identity_matches << ",\n"
           << "      \"unmatched_tracks\": " << result.unmatched_tracks << ",\n"
           << "      \"identity_position_rmse_m\": "
           << result.identity_position_rmse_m << ",\n"
           << "      \"mean_assignment_time_us\": "
           << result.mean_assignment_time_us << "\n"
           << "    }" << (index + 1U == results.size() ? "\n" : ",\n");
  }
  output << "  ],\n"
         << "  \"claim_boundary\": \"Deterministic heteroscedastic association A/B; "
         << "it validates covariance use but not physical perception accuracy.\"\n"
         << "}\n";
}

}  // namespace
}  // namespace embodied_navigation

int main(int argc, char ** argv)
{
  std::string output_path;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--output" && index + 1 < argc) {
      output_path = argv[++index];
    } else {
      std::cerr << "usage: dynamic_obstacle_uncertainty_benchmark [--output REPORT.json]\n";
      return 2;
    }
  }
  const std::vector<embodied_navigation::MetricResult> results{
    embodied_navigation::run_metric(embodied_navigation::AssociationMetric::Euclidean),
    embodied_navigation::run_metric(embodied_navigation::AssociationMetric::Mahalanobis)};
  if (output_path.empty()) {
    embodied_navigation::write_report(std::cout, results);
    return 0;
  }
  std::ofstream output(output_path);
  if (!output) {
    std::cerr << "failed to open report: " << output_path << '\n';
    return 2;
  }
  embodied_navigation::write_report(output, results);
  std::cout << "dynamic obstacle uncertainty report: " << output_path << '\n';
  return 0;
}
