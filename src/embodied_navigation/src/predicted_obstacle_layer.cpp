#include "embodied_navigation/predicted_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace embodied_navigation
{

void PredictedObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PredictedObstacleLayer lifecycle node expired");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("prediction_horizon_s", rclcpp::ParameterValue(prediction_horizon_s_));
  declareParameter("prediction_step_s", rclcpp::ParameterValue(prediction_step_s_));
  declareParameter("observation_timeout_s", rclcpp::ParameterValue(observation_timeout_s_));
  declareParameter("minimum_confidence", rclcpp::ParameterValue(minimum_confidence_));
  declareParameter("radius_padding_m", rclcpp::ParameterValue(radius_padding_m_));
  declareParameter("future_radius_growth_mps", rclcpp::ParameterValue(future_radius_growth_mps_));
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("topic"), topic_);
  node->get_parameter(getFullName("prediction_horizon_s"), prediction_horizon_s_);
  node->get_parameter(getFullName("prediction_step_s"), prediction_step_s_);
  node->get_parameter(getFullName("observation_timeout_s"), observation_timeout_s_);
  node->get_parameter(getFullName("minimum_confidence"), minimum_confidence_);
  node->get_parameter(getFullName("radius_padding_m"), radius_padding_m_);
  node->get_parameter(getFullName("future_radius_growth_mps"), future_radius_growth_mps_);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  subscription_ = node->create_subscription<embodied_agent_interfaces::msg::DynamicObstacleArray>(
    topic_, rclcpp::QoS(10).reliable(),
    std::bind(&PredictedObstacleLayer::on_obstacles, this, std::placeholders::_1), options);
  current_ = true;
  RCLCPP_INFO(
    logger_, "Predicted obstacle layer: topic=%s horizon=%.2fs step=%.2fs",
    topic_.c_str(), prediction_horizon_s_, prediction_step_s_);
}

void PredictedObstacleLayer::on_obstacles(
  const embodied_agent_interfaces::msg::DynamicObstacleArray::SharedPtr message)
{
  if (message->header.frame_id != layered_costmap_->getGlobalFrameID()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3000, "Ignoring dynamic obstacles in frame '%s'; expected '%s'",
      message->header.frame_id.c_str(), layered_costmap_->getGlobalFrameID().c_str());
    return;
  }
  std::scoped_lock lock(mutex_);
  std::vector<ObstacleMotion> obstacles;
  obstacles.reserve(message->obstacles.size());
  for (const auto & obstacle : message->obstacles) {
    obstacles.push_back(ObstacleMotion{
      obstacle.position.x,
      obstacle.position.y,
      obstacle.velocity.x,
      obstacle.velocity.y,
      obstacle.radius,
      obstacle.confidence});
  }
  active_points_ = predict_constant_velocity(obstacles, prediction_config());
  last_observation_time_ = clock_->now();
  current_ = true;
}

PredictionConfig PredictedObstacleLayer::prediction_config() const
{
  return PredictionConfig{
    prediction_horizon_s_, prediction_step_s_, minimum_confidence_, radius_padding_m_,
    future_radius_growth_mps_};
}

void PredictedObstacleLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  std::scoped_lock lock(mutex_);
  std::vector<PredictedPoint> next = active_points_;
  if (last_observation_time_.nanoseconds() == 0 ||
    (clock_->now() - last_observation_time_).seconds() > observation_timeout_s_)
  {
    // 超时后清空预测；旧范围仍参与本轮 bounds，使 master costmap 能擦除残留代价。
    // “没有动态障碍”是有效的空观测，不能把 Layer 标成 stale，否则 Nav2 会拒绝规划。
    next.clear();
    current_ = true;
  }
  for (const auto & point : previous_points_) {
    *min_x = std::min(*min_x, point.x - point.radius);
    *min_y = std::min(*min_y, point.y - point.radius);
    *max_x = std::max(*max_x, point.x + point.radius);
    *max_y = std::max(*max_y, point.y + point.radius);
  }
  for (const auto & point : next) {
    *min_x = std::min(*min_x, point.x - point.radius);
    *min_y = std::min(*min_y, point.y - point.radius);
    *max_x = std::max(*max_x, point.x + point.radius);
    *max_y = std::max(*max_y, point.y + point.radius);
  }
  active_points_ = next;
  previous_points_ = next;
}

void PredictedObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  std::scoped_lock lock(mutex_);
  for (const auto & point : active_points_) {
    unsigned int center_x = 0U;
    unsigned int center_y = 0U;
    if (!master_grid.worldToMap(point.x, point.y, center_x, center_y)) {
      continue;
    }
    const int cell_radius = static_cast<int>(std::ceil(point.radius / master_grid.getResolution()));
    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        if (dx * dx + dy * dy > cell_radius * cell_radius) {
          continue;
        }
        const int mx = static_cast<int>(center_x) + dx;
        const int my = static_cast<int>(center_y) + dy;
        if (mx < min_i || my < min_j || mx >= max_i || my >= max_j || mx < 0 || my < 0 ||
          mx >= static_cast<int>(master_grid.getSizeInCellsX()) ||
          my >= static_cast<int>(master_grid.getSizeInCellsY()))
        {
          continue;
        }
        master_grid.setCost(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void PredictedObstacleLayer::reset()
{
  std::scoped_lock lock(mutex_);
  active_points_.clear();
  previous_points_.clear();
  current_ = true;
}

}  // namespace embodied_navigation

PLUGINLIB_EXPORT_CLASS(embodied_navigation::PredictedObstacleLayer, nav2_costmap_2d::Layer)
