#include "embodied_slam/lidar_loop_verifier.hpp"

#include <cmath>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace embodied_slam
{

LiveLidarMatchingMode liveLidarMatchingModeFromString(const std::string & value)
{
  if (value == "scan_to_scan") {
    return LiveLidarMatchingMode::ScanToScan;
  }
  if (value == "scan_to_submap") {
    return LiveLidarMatchingMode::ScanToSubmap;
  }
  throw std::invalid_argument("matching_mode must be scan_to_scan or scan_to_submap");
}

std::string toString(const LiveLidarMatchingMode mode)
{
  return mode == LiveLidarMatchingMode::ScanToSubmap ? "scan_to_submap" : "scan_to_scan";
}

LiveLidarLoopVerifier::LiveLidarLoopVerifier(const LiveLidarLoopVerifierConfig & config)
: config_(config)
{
  if (config_.maximum_cached_scans == 0U || config_.maximum_cached_odometry == 0U ||
    config_.maximum_odometry_time_delta_ns <= 0)
  {
    throw std::invalid_argument("live lidar cache sizes and odometry tolerance must be positive");
  }
  if (config_.matching_mode == LiveLidarMatchingMode::ScanToSubmap) {
    // 复用离线/在线共享构建器的校验，防止两条入口接受不同的非法参数。
    (void)assembleLidarSubmap(0, {}, config_.submap);
  }
}

void LiveLidarLoopVerifier::cacheScan(
  const std::int64_t stamp_ns, std::vector<LidarPoint2D> points)
{
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("cached scan points must be finite");
    }
  }

  const auto existing = scans_.find(stamp_ns);
  if (existing != scans_.end()) {
    // 同一时间戳可能由 bag 重播重复送达；更新几何但不重复占用淘汰队列。
    existing->second.points = std::move(points);
    return;
  }
  scans_.emplace(stamp_ns, CachedScan{std::move(points)});
  scan_insertion_order_.push_back(stamp_ns);
  while (scans_.size() > config_.maximum_cached_scans) {
    scans_.erase(scan_insertion_order_.front());
    scan_insertion_order_.pop_front();
  }
}

void LiveLidarLoopVerifier::cacheOdometry(const std::int64_t stamp_ns, Pose2d pose)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
    throw std::invalid_argument("cached odometry pose must be finite");
  }
  pose.yaw = normalize_angle(pose.yaw);
  const auto existing = odometry_.find(stamp_ns);
  if (existing != odometry_.end()) {
    existing->second = pose;
    return;
  }
  odometry_.emplace(stamp_ns, pose);
  odometry_insertion_order_.push_back(stamp_ns);
  while (odometry_.size() > config_.maximum_cached_odometry) {
    odometry_.erase(odometry_insertion_order_.front());
    odometry_insertion_order_.pop_front();
  }
}

bool LiveLidarLoopVerifier::hasScan(const std::int64_t stamp_ns) const
{
  return scans_.find(stamp_ns) != scans_.end();
}

bool LiveLidarLoopVerifier::hasGeometry(const std::int64_t stamp_ns) const
{
  if (config_.matching_mode == LiveLidarMatchingMode::ScanToScan) {
    return hasScan(stamp_ns);
  }
  return buildSubmap(stamp_ns).available;
}

std::size_t LiveLidarLoopVerifier::cachedScans() const
{
  return scans_.size();
}

std::size_t LiveLidarLoopVerifier::cachedOdometry() const
{
  return odometry_.size();
}

LiveLidarMatchingMode LiveLidarLoopVerifier::matchingMode() const
{
  return config_.matching_mode;
}

std::optional<Pose2d> LiveLidarLoopVerifier::nearestOdometry(
  const std::int64_t stamp_ns) const
{
  if (odometry_.empty()) {
    return std::nullopt;
  }
  auto best = odometry_.end();
  std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();
  const auto after = odometry_.lower_bound(stamp_ns);
  if (after != odometry_.end()) {
    best = after;
    best_delta = std::llabs(after->first - stamp_ns);
  }
  if (after != odometry_.begin()) {
    const auto before = std::prev(after);
    const auto delta = std::llabs(before->first - stamp_ns);
    if (delta <= best_delta) {
      best = before;
      best_delta = delta;
    }
  }
  if (best == odometry_.end() || best_delta > config_.maximum_odometry_time_delta_ns) {
    return std::nullopt;
  }
  return best->second;
}

LidarSubmapGeometry LiveLidarLoopVerifier::buildSubmap(
  const std::int64_t center_stamp_ns) const
{
  const auto center = scans_.find(center_stamp_ns);
  if (center == scans_.end()) {
    LidarSubmapGeometry missing;
    missing.center_frame_id = center_stamp_ns;
    missing.rejection_reason = "missing_center_scan_or_odometry";
    return missing;
  }

  auto first = center;
  for (std::size_t count = 0U;
    count < config_.submap.half_window_scans && first != scans_.begin(); ++count)
  {
    --first;
  }
  auto last = std::next(center);
  for (std::size_t count = 0U;
    count < config_.submap.half_window_scans && last != scans_.end(); ++count)
  {
    ++last;
  }

  std::vector<LidarSubmapFrameView> frames;
  frames.reserve(config_.submap.half_window_scans * 2U + 1U);
  for (auto scan = first; scan != last; ++scan) {
    const auto odometry = nearestOdometry(scan->first);
    if (!odometry) {
      continue;
    }
    frames.push_back({
      scan->first, static_cast<double>(scan->first) / 1.0e9,
      &scan->second.points, *odometry});
  }
  return assembleLidarSubmap(center_stamp_ns, frames, config_.submap);
}

std::vector<LiveLidarLoopVerification> LiveLidarLoopVerifier::verify(
  const std::int64_t query_stamp_ns,
  const std::vector<LiveLidarLoopCandidateInput> & candidates) const
{
  std::vector<LiveLidarLoopVerification> output;
  output.reserve(candidates.size());
  const auto query = scans_.find(query_stamp_ns);
  const auto query_submap = config_.matching_mode == LiveLidarMatchingMode::ScanToSubmap ?
    buildSubmap(query_stamp_ns) : LidarSubmapGeometry{};
  for (const auto & candidate : candidates) {
    LiveLidarLoopVerification verification;
    verification.candidate = candidate;
    if (query != scans_.end() && config_.matching_mode == LiveLidarMatchingMode::ScanToScan) {
      verification.query_contributing_scans = 1U;
    }
    if (query == scans_.end()) {
      verification.match.rejection_reason = "query_scan_not_cached";
    } else if (config_.matching_mode == LiveLidarMatchingMode::ScanToSubmap &&
      !query_submap.available)
    {
      verification.query_contributing_scans = query_submap.contributing_frame_ids.size();
      verification.match.rejection_reason = "query_submap_" + query_submap.rejection_reason;
    } else {
      const auto target = scans_.find(candidate.candidate_stamp_ns);
      if (target == scans_.end()) {
        verification.match.rejection_reason = "candidate_scan_not_cached";
      } else if (config_.matching_mode == LiveLidarMatchingMode::ScanToSubmap) {
        const auto candidate_submap = buildSubmap(candidate.candidate_stamp_ns);
        verification.query_contributing_scans = query_submap.contributing_frame_ids.size();
        verification.candidate_contributing_scans =
          candidate_submap.contributing_frame_ids.size();
        if (!candidate_submap.available) {
          verification.match.rejection_reason =
            "candidate_submap_" + candidate_submap.rejection_reason;
        } else {
          verification.match = matchLidarScans(
            query_submap.points, candidate_submap.points,
            candidate.yaw_offset_rad, config_.matcher);
        }
      } else {
        verification.candidate_contributing_scans = 1U;
        verification.match = matchLidarScans(
          query->second.points, target->second.points,
          candidate.yaw_offset_rad, config_.matcher);
      }
    }
    output.push_back(std::move(verification));
  }
  return output;
}

}  // namespace embodied_slam
