#include "embodied_navigation/dynamic_obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace embodied_navigation
{

DynamicObstacleTracker::DynamicObstacleTracker(TrackerConfig config)
: config_(std::move(config))
{
}

const std::vector<TrackedObstacle> & DynamicObstacleTracker::update(
  const std::vector<Point2d> & observations, double timestamp_s)
{
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this, timestamp_s](const auto & track) {
        return timestamp_s - track.last_seen_s > config_.track_timeout_s;
      }),
    tracks_.end());

  std::vector<bool> observation_used(observations.size(), false);
  for (auto & track : tracks_) {
    std::size_t best_index = observations.size();
    double best_distance = config_.association_distance_m;
    for (std::size_t index = 0; index < observations.size(); ++index) {
      if (observation_used[index]) {
        continue;
      }
      const double distance = std::hypot(
        observations[index].x - track.position.x, observations[index].y - track.position.y);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = index;
      }
    }
    if (best_index == observations.size()) {
      continue;
    }
    const double delta_s = timestamp_s - track.last_seen_s;
    if (delta_s > 1e-4) {
      const Point2d measured_velocity{
        (observations[best_index].x - track.position.x) / delta_s,
        (observations[best_index].y - track.position.y) / delta_s};
      const double alpha = std::clamp(config_.velocity_smoothing, 0.0, 1.0);
      track.velocity.x = alpha * measured_velocity.x + (1.0 - alpha) * track.velocity.x;
      track.velocity.y = alpha * measured_velocity.y + (1.0 - alpha) * track.velocity.y;
    }
    track.position = observations[best_index];
    track.last_seen_s = timestamp_s;
    ++track.observation_count;
    track.confidence = std::min(1.0, 0.45 + 0.12 * static_cast<double>(track.observation_count));
    observation_used[best_index] = true;
  }

  for (std::size_t index = 0; index < observations.size(); ++index) {
    if (observation_used[index]) {
      continue;
    }
    TrackedObstacle track;
    track.id = "track_" + std::to_string(next_track_id_++);
    track.position = observations[index];
    track.radius = config_.default_radius_m;
    track.last_seen_s = timestamp_s;
    track.observation_count = 1U;
    tracks_.push_back(std::move(track));
  }
  return tracks_;
}

const std::vector<TrackedObstacle> & DynamicObstacleTracker::tracks() const
{
  return tracks_;
}

void DynamicObstacleTracker::clear()
{
  tracks_.clear();
  next_track_id_ = 1U;
}

}  // namespace embodied_navigation
