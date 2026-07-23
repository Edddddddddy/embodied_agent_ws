#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "embodied_navigation/dynamic_obstacle_tracker.hpp"

namespace embodied_navigation
{
namespace
{

struct TruthState
{
  Point2d position;
  Point2d velocity;
};

struct Metrics
{
  std::string model;
  std::size_t samples{0U};
  std::size_t occlusion_samples{0U};
  std::size_t track_drop_count{0U};
  double position_squared_error{0.0};
  double velocity_squared_error{0.0};
  double forecast_squared_error{0.0};
  double occlusion_squared_error{0.0};
  double stop_forecast_squared_error{0.0};
  std::size_t stop_forecast_samples{0U};
  double update_time_us{0.0};
};

TruthState truth_at(double time_s)
{
  if (time_s < 2.0) {
    return {{0.0, 0.0}, {0.0, 0.0}};
  }
  if (time_s < 5.0) {
    return {{0.6 * (time_s - 2.0), 0.0}, {0.6, 0.0}};
  }
  if (time_s < 7.0) {
    return {{1.8, 0.6 * (time_s - 5.0)}, {0.0, 0.6}};
  }
  return {{1.8, 1.2}, {0.0, 0.0}};
}

Point2d deterministic_measurement(const TruthState & truth, double time_s)
{
  return {
    truth.position.x + 0.04 * std::sin(5.3 * time_s) + 0.015 * std::cos(11.7 * time_s),
    truth.position.y + 0.035 * std::cos(4.7 * time_s) - 0.012 * std::sin(9.1 * time_s)};
}

double squared_distance(const Point2d & first, const Point2d & second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  return dx * dx + dy * dy;
}

double rmse(double squared_error, std::size_t samples)
{
  return samples == 0U ? 0.0 : std::sqrt(squared_error / static_cast<double>(samples));
}

Metrics run_model(MotionModel model)
{
  TrackerConfig config;
  config.motion_model = model;
  config.association_distance_m = 1.2;
  config.track_timeout_s = 1.0;
  config.velocity_smoothing = 0.5;
  config.measurement_noise_variance = 0.0016;
  config.process_noise_variance = 0.35;
  config.imm_stationary_process_noise = 0.004;
  config.imm_maneuver_process_noise = 1.2;
  config.imm_stay_probability = 0.94;
  config.imm_stationary_velocity_decay = 0.1;
  DynamicObstacleTracker tracker(config);

  Metrics metrics;
  metrics.model = to_string(model);
  constexpr double step_s = 0.1;
  constexpr double forecast_horizon_s = 0.75;
  for (int index = 0; index <= 90; ++index) {
    const double time_s = step_s * static_cast<double>(index);
    const TruthState truth = truth_at(time_s);
    const bool occluded = time_s >= 5.5 && time_s <= 6.1;
    const std::vector<Point2d> observations = occluded ?
      std::vector<Point2d>{} : std::vector<Point2d>{deterministic_measurement(truth, time_s)};

    const auto started = std::chrono::steady_clock::now();
    const auto & tracks = tracker.update(observations, time_s);
    const auto finished = std::chrono::steady_clock::now();
    metrics.update_time_us += std::chrono::duration<double, std::micro>(finished - started).count();
    if (tracks.empty()) {
      ++metrics.track_drop_count;
      continue;
    }

    const auto & track = tracks.front();
    ++metrics.samples;
    metrics.position_squared_error += squared_distance(track.position, truth.position);
    if (time_s >= 0.5) {
      metrics.velocity_squared_error += squared_distance(track.velocity, truth.velocity);
    }
    const Point2d forecast{
      track.position.x + track.velocity.x * forecast_horizon_s,
      track.position.y + track.velocity.y * forecast_horizon_s};
    const TruthState future_truth = truth_at(time_s + forecast_horizon_s);
    metrics.forecast_squared_error += squared_distance(forecast, future_truth.position);
    if (occluded) {
      ++metrics.occlusion_samples;
      metrics.occlusion_squared_error += squared_distance(track.position, truth.position);
    }
    if (time_s >= 7.3) {
      ++metrics.stop_forecast_samples;
      metrics.stop_forecast_squared_error += squared_distance(forecast, future_truth.position);
    }
  }
  return metrics;
}

void write_report(std::ostream & output, const std::vector<Metrics> & results)
{
  output << std::fixed << std::setprecision(8);
  output << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"scenario\": \"stop_turn_occlusion_v1\",\n"
         << "  \"measurement_noise\": \"deterministic_sine_mix\",\n"
         << "  \"sample_step_s\": 0.1,\n"
         << "  \"forecast_horizon_s\": 0.75,\n"
         << "  \"occlusion_interval_s\": [5.5, 6.1],\n"
         << "  \"tracker_config\": {\n"
         << "    \"association_distance_m\": 1.2,\n"
         << "    \"association_strategy\": \"global_nearest\",\n"
         << "    \"association_metric\": \"euclidean\",\n"
         << "    \"association_nis_gate\": 9.210,\n"
         << "    \"track_timeout_s\": 1.0,\n"
         << "    \"velocity_smoothing\": 0.5,\n"
         << "    \"measurement_noise_variance\": 0.0016,\n"
         << "    \"process_noise_variance\": 0.35,\n"
         << "    \"imm_stationary_process_noise\": 0.004,\n"
         << "    \"imm_maneuver_process_noise\": 1.2,\n"
         << "    \"imm_stay_probability\": 0.94,\n"
         << "    \"imm_stationary_velocity_decay\": 0.1\n"
         << "  },\n"
         << "  \"models\": [\n";
  for (std::size_t index = 0; index < results.size(); ++index) {
    const auto & result = results[index];
    const std::size_t velocity_samples = result.samples > 5U ? result.samples - 5U : result.samples;
    output << "    {\n"
           << "      \"model\": \"" << result.model << "\",\n"
           << "      \"samples\": " << result.samples << ",\n"
           << "      \"occlusion_samples\": " << result.occlusion_samples << ",\n"
           << "      \"track_drop_count\": " << result.track_drop_count << ",\n"
           << "      \"position_rmse_m\": " << rmse(result.position_squared_error, result.samples) << ",\n"
           << "      \"velocity_rmse_mps\": " << rmse(result.velocity_squared_error, velocity_samples) << ",\n"
           << "      \"forecast_rmse_m\": " << rmse(result.forecast_squared_error, result.samples) << ",\n"
           << "      \"occlusion_rmse_m\": " << rmse(result.occlusion_squared_error, result.occlusion_samples) << ",\n"
           << "      \"stop_forecast_rmse_m\": " << rmse(result.stop_forecast_squared_error, result.stop_forecast_samples) << ",\n"
           << "      \"mean_update_time_us\": " << result.update_time_us / std::max<std::size_t>(1U, result.samples) << "\n"
           << "    }" << (index + 1U == results.size() ? "\n" : ",\n");
  }
  output << "  ],\n"
         << "  \"claim_boundary\": \"Deterministic tracker-level ablation; it does not replace Gazebo navigation success evidence.\"\n"
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
      std::cerr << "usage: dynamic_obstacle_model_benchmark [--output REPORT.json]\n";
      return 2;
    }
  }

  const std::vector<embodied_navigation::Metrics> results{
    embodied_navigation::run_model(embodied_navigation::MotionModel::CurrentOnly),
    embodied_navigation::run_model(embodied_navigation::MotionModel::ConstantVelocity),
    embodied_navigation::run_model(embodied_navigation::MotionModel::Kalman),
    embodied_navigation::run_model(embodied_navigation::MotionModel::Imm)};

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
  std::cout << "dynamic obstacle model report: " << output_path << '\n';
  return 0;
}
