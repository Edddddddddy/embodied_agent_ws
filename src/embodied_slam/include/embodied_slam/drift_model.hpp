#pragma once

#include <cstdint>
#include <random>

#include "embodied_slam/pose2d.hpp"

namespace embodied_slam
{

struct DriftConfig
{
  double linear_scale{1.04};
  double lateral_scale{1.0};
  double yaw_bias_per_meter{0.035};
  double translation_noise_stddev{0.002};
  double yaw_noise_stddev{0.001};
  std::uint32_t random_seed{42U};
};

class DriftModel
{
public:
  explicit DriftModel(DriftConfig config = {});

  Pose2d update(const Pose2d & reference_pose);
  void reset();
  bool initialized() const;
  const Pose2d & estimate() const;

private:
  DriftConfig config_;
  std::mt19937 random_engine_;
  std::normal_distribution<double> unit_normal_{0.0, 1.0};
  bool initialized_{false};
  Pose2d previous_reference_;
  Pose2d estimate_;
};

}  // namespace embodied_slam
