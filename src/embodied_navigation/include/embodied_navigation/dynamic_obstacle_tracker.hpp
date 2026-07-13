#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace embodied_navigation
{

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

struct TrackedObstacle
{
  std::string id;
  Point2d position;
  Point2d velocity;
  double radius{0.25};
  double confidence{0.5};
  double last_seen_s{0.0};
  std::size_t observation_count{0U};
};

struct TrackerConfig
{
  double association_distance_m{0.8};
  double velocity_smoothing{0.65};
  double track_timeout_s{1.0};
  double default_radius_m{0.25};
};

class DynamicObstacleTracker
{
public:
  explicit DynamicObstacleTracker(TrackerConfig config = {});
  const std::vector<TrackedObstacle> & update(
    const std::vector<Point2d> & observations, double timestamp_s);
  const std::vector<TrackedObstacle> & tracks() const;
  void clear();

private:
  TrackerConfig config_;
  std::vector<TrackedObstacle> tracks_;
  std::size_t next_track_id_{1U};
};

}  // namespace embodied_navigation
