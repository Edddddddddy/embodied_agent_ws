#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "embodied_slam/lidar_scan_matcher.hpp"
#include "embodied_slam/lidar_submap_builder.hpp"

namespace embodied_slam
{

enum class LiveLidarMatchingMode
{
  ScanToScan,
  ScanToSubmap
};

struct LiveLidarLoopVerifierConfig
{
  std::size_t maximum_cached_scans{12000U};
  std::size_t maximum_cached_odometry{12000U};
  std::int64_t maximum_odometry_time_delta_ns{50000000LL};
  LiveLidarMatchingMode matching_mode{LiveLidarMatchingMode::ScanToSubmap};
  LidarScanMatcherConfig matcher;
  LidarSubmapConfig submap;
};

struct LiveLidarLoopCandidateInput
{
  std::int64_t candidate_id{0};
  std::int64_t candidate_stamp_ns{0};
  std::uint32_t rank{0U};
  double similarity{0.0};
  double yaw_offset_rad{0.0};
};

struct LiveLidarLoopVerification
{
  LiveLidarLoopCandidateInput candidate;
  std::size_t query_contributing_scans{0U};
  std::size_t candidate_contributing_scans{0U};
  LidarScanMatchResult match;
};

LiveLidarMatchingMode liveLidarMatchingModeFromString(const std::string & value);
std::string toString(LiveLidarMatchingMode mode);

// 该领域对象不依赖 ROS 消息：节点只负责时间戳关联和发布，几何门限、缓存淘汰
// 与拒绝原因都集中在这里，因而可以用确定性的 C++ 单元测试覆盖。
class LiveLidarLoopVerifier
{
public:
  explicit LiveLidarLoopVerifier(const LiveLidarLoopVerifierConfig & config = {});

  void cacheScan(std::int64_t stamp_ns, std::vector<LidarPoint2D> points);
  void cacheOdometry(std::int64_t stamp_ns, Pose2d pose);
  bool hasScan(std::int64_t stamp_ns) const;
  bool hasGeometry(std::int64_t stamp_ns) const;
  std::size_t cachedScans() const;
  std::size_t cachedOdometry() const;
  LiveLidarMatchingMode matchingMode() const;

  std::vector<LiveLidarLoopVerification> verify(
    std::int64_t query_stamp_ns,
    const std::vector<LiveLidarLoopCandidateInput> & candidates) const;

private:
  struct CachedScan
  {
    std::vector<LidarPoint2D> points;
  };

  std::optional<Pose2d> nearestOdometry(std::int64_t stamp_ns) const;
  LidarSubmapGeometry buildSubmap(std::int64_t center_stamp_ns) const;

  LiveLidarLoopVerifierConfig config_;
  std::map<std::int64_t, CachedScan> scans_;
  std::map<std::int64_t, Pose2d> odometry_;
  std::deque<std::int64_t> scan_insertion_order_;
  std::deque<std::int64_t> odometry_insertion_order_;
};

}  // namespace embodied_slam
