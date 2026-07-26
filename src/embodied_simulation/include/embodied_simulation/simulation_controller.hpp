#pragma once

#include <limits>
#include <string>
#include <vector>

namespace embodied_simulation
{

enum class ControlMode
{
  kManual,
  kObstacleAvoidance,
  kWallFollowing,
};

struct ControllerConfig
{
  double max_linear_speed{0.22};
  double max_angular_speed{1.0};
  double linear_acceleration{0.5};
  double angular_acceleration{2.0};
  double emergency_distance{0.22};
  double obstacle_distance{0.50};
  double wall_target_distance{0.40};
  double autonomous_linear_speed{0.14};
  double obstacle_turn_speed{0.65};
  double scan_timeout{0.50};
  double stop_timeout_s{3.0};
  double wall_kp{1.8};
  double wall_ki{0.0};
  double wall_kd{0.15};
};

struct VelocityCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct ControllerOutput
{
  VelocityCommand velocity;
  ControlMode mode{ControlMode::kManual};
  bool sensor_stale{true};
  bool safety_stopped{false};
  double front_distance{std::numeric_limits<double>::infinity()};
  double right_distance{std::numeric_limits<double>::infinity()};
  std::string reason{"waiting_for_scan"};
};

class SimulationController
{
public:
  explicit SimulationController(ControllerConfig config = {});

  bool set_mode(const std::string & mode);
  ControlMode mode() const;
  static std::string mode_name(ControlMode mode);

  void set_manual_command(
    double linear_x, double angular_z, double duration_s, double now_s);
  void stop();
  void set_emergency_stop(bool active);
  void update_scan(
    const std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max,
    double now_s);
  ControllerOutput step(double now_s);

private:
  static double approach(double current, double target, double max_delta);
  static double normalized_angle(double angle);
  static bool in_sector(double angle, double center, double half_width);
  static double clamp(double value, double lower, double upper);
  void reset_pid();

  ControllerConfig config_;
  ControlMode mode_{ControlMode::kManual};
  VelocityCommand manual_command_;
  VelocityCommand current_command_;
  double manual_until_s_{0.0};
  double last_step_s_{std::numeric_limits<double>::quiet_NaN()};
  double last_scan_s_{-std::numeric_limits<double>::infinity()};
  double front_distance_{std::numeric_limits<double>::infinity()};
  double left_distance_{std::numeric_limits<double>::infinity()};
  double right_distance_{std::numeric_limits<double>::infinity()};
  bool emergency_stop_{false};
  bool force_stop_{false};
  double wall_integral_{0.0};
  double wall_previous_error_{0.0};
};

}  // namespace embodied_simulation
