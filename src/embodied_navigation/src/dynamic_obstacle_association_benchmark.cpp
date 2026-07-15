#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "embodied_navigation/dynamic_obstacle_tracker.hpp"

namespace embodied_navigation
{
namespace
{

struct AssociationMetrics
{
  AssociationStrategy strategy{AssociationStrategy::GreedyNearest};
  std::size_t total_tracks{0U};
  std::size_t matched_existing_tracks{0U};
  std::size_t fragment_tracks{0U};
  double identity_position_rmse_m{0.0};
  double update_time_us{0.0};
};

AssociationMetrics run_association(const AssociationStrategy strategy)
{
  TrackerConfig config;
  config.motion_model = MotionModel::CurrentOnly;
  config.association_strategy = strategy;
  config.association_distance_m = 0.5;
  DynamicObstacleTracker tracker(config);
  tracker.update({Point2d{0.0, 0.0}, Point2d{0.3, 0.0}}, 0.0);

  // 观测顺序故意让第一条轨迹遇到等距冲突：贪心会占走 track_2 唯一可用观测，
  // 全局分配则同时最小化两条创新距离。输入不依赖真值调参，可重复运行。
  const auto started = std::chrono::steady_clock::now();
  const auto & tracks = tracker.update(
    {Point2d{0.2, 0.0}, Point2d{-0.2, 0.0}}, 0.1);
  const auto finished = std::chrono::steady_clock::now();

  AssociationMetrics metrics;
  metrics.strategy = strategy;
  metrics.total_tracks = tracks.size();
  metrics.update_time_us =
    std::chrono::duration<double, std::micro>(finished - started).count();
  double squared_error = 0.0;
  std::size_t identity_samples = 0U;
  for (const auto & track : tracks) {
    if (track.id == "track_1" || track.id == "track_2") {
      ++identity_samples;
      metrics.matched_existing_tracks += track.observation_count == 2U ? 1U : 0U;
      const double expected_x = track.id == "track_1" ? -0.2 : 0.2;
      const double error = track.position.x - expected_x;
      squared_error += error * error + track.position.y * track.position.y;
    } else {
      ++metrics.fragment_tracks;
    }
  }
  metrics.identity_position_rmse_m = identity_samples == 0U ? 0.0 :
    std::sqrt(squared_error / static_cast<double>(identity_samples));
  return metrics;
}

void write_report(std::ostream & output, const std::vector<AssociationMetrics> & results)
{
  output << std::fixed << std::setprecision(8)
         << "{\n"
         << "  \"schema_version\": 1,\n"
         << "  \"scenario\": \"two_track_conflicting_gate_v1\",\n"
         << "  \"association_distance_m\": 0.5,\n"
         << "  \"motion_model\": \"current_only\",\n"
         << "  \"strategies\": [\n";
  for (std::size_t index = 0U; index < results.size(); ++index) {
    const auto & result = results[index];
    output << "    {\n"
           << "      \"strategy\": \"" << to_string(result.strategy) << "\",\n"
           << "      \"total_tracks\": " << result.total_tracks << ",\n"
           << "      \"matched_existing_tracks\": "
           << result.matched_existing_tracks << ",\n"
           << "      \"fragment_tracks\": " << result.fragment_tracks << ",\n"
           << "      \"identity_position_rmse_m\": "
           << result.identity_position_rmse_m << ",\n"
           << "      \"update_time_us\": " << result.update_time_us << "\n"
           << "    }" << (index + 1U == results.size() ? "\n" : ",\n");
  }
  output << "  ],\n"
         << "  \"claim_boundary\": \"Deterministic association-level conflict; "
         << "it demonstrates order independence but not perception accuracy.\"\n"
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
      std::cerr << "usage: dynamic_obstacle_association_benchmark [--output REPORT.json]\n";
      return 2;
    }
  }
  const std::vector<embodied_navigation::AssociationMetrics> results{
    embodied_navigation::run_association(embodied_navigation::AssociationStrategy::GreedyNearest),
    embodied_navigation::run_association(embodied_navigation::AssociationStrategy::GlobalNearest)};
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
  std::cout << "dynamic obstacle association report: " << output_path << '\n';
  return 0;
}
