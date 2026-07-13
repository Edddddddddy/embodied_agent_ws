#include "embodied_navigation/constant_velocity_predictor.hpp"

#include <algorithm>

namespace embodied_navigation
{

std::vector<PredictedPoint> predict_constant_velocity(
  const std::vector<ObstacleMotion> & obstacles, const PredictionConfig & config)
{
  std::vector<PredictedPoint> output;
  const double horizon_s = std::max(0.0, config.horizon_s);
  const double step_s = std::max(0.05, config.step_s);
  for (const auto & obstacle : obstacles) {
    if (obstacle.confidence < config.minimum_confidence) {
      continue;
    }
    for (double time_s = 0.0; time_s <= horizon_s + 1e-6; time_s += step_s) {
      // 越远期的预测越不确定，因此逐步膨胀安全半径，主动给规划器留出余量。
      const double radius = std::max(
        0.05, obstacle.radius + config.radius_padding_m +
        config.future_radius_growth_mps * time_s);
      output.push_back(PredictedPoint{
        obstacle.x + obstacle.velocity_x * time_s,
        obstacle.y + obstacle.velocity_y * time_s,
        radius,
        time_s});
    }
  }
  return output;
}

}  // namespace embodied_navigation
