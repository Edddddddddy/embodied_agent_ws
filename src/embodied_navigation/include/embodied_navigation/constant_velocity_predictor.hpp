#pragma once

#include <vector>

namespace embodied_navigation
{

struct ObstacleMotion
{
  double x{0.0};
  double y{0.0};
  double velocity_x{0.0};
  double velocity_y{0.0};
  double radius{0.0};
  double confidence{0.0};
};

struct PredictedPoint
{
  double x{0.0};
  double y{0.0};
  double radius{0.0};
  double time_s{0.0};
};

struct PredictionConfig
{
  double horizon_s{2.0};
  double step_s{0.25};
  double minimum_confidence{0.45};
  double radius_padding_m{0.18};
  double future_radius_growth_mps{0.08};
};

/// 使用常速度模型生成未来占用圆。该纯函数不依赖 ROS，便于单元测试和替换为更复杂模型。
std::vector<PredictedPoint> predict_constant_velocity(
  const std::vector<ObstacleMotion> & obstacles, const PredictionConfig & config);

}  // namespace embodied_navigation
