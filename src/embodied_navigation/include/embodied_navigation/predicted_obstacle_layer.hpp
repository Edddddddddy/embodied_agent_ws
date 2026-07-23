#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/dynamic_obstacle_array.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "embodied_navigation/constant_velocity_predictor.hpp"

namespace embodied_navigation
{

class PredictedObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  void on_obstacles(const embodied_agent_interfaces::msg::DynamicObstacleArray::SharedPtr message);
  PredictionConfig prediction_config() const;

  std::mutex mutex_;
  std::vector<PredictedPoint> active_points_;
  std::vector<PredictedPoint> previous_points_;
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<embodied_agent_interfaces::msg::DynamicObstacleArray>::SharedPtr subscription_;
  std::string topic_{"/perception/dynamic_obstacles"};
  double prediction_horizon_s_{2.0};
  double prediction_step_s_{0.25};
  double observation_timeout_s_{0.8};
  double minimum_confidence_{0.45};
  double radius_padding_m_{0.18};
  double future_radius_growth_mps_{0.08};
};

}  // namespace embodied_navigation
