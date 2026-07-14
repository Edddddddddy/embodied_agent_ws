#include "embodied_slam/scan_overlap_validator.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace embodied_slam
{
namespace
{

struct GridKey
{
  std::int64_t x{0};
  std::int64_t y{0};

  bool operator==(const GridKey & other) const {return x == other.x && y == other.y;}
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & key) const
  {
    const auto x = static_cast<std::uint64_t>(key.x);
    const auto y = static_cast<std::uint64_t>(key.y);
    return static_cast<std::size_t>(x * 0x9e3779b185ebca87ULL ^ (y + 0x85ebca77c2b2ae63ULL));
  }
};

using SpatialGrid = std::unordered_map<GridKey, std::vector<ScanPoint2d>, GridKeyHash>;

std::vector<ScanPoint2d> sampleFinite(
  const std::vector<ScanPoint2d> & input, const std::size_t stride)
{
  std::vector<ScanPoint2d> output;
  output.reserve((input.size() + stride - 1U) / stride);
  for (std::size_t index = 0; index < input.size(); index += stride) {
    if (std::isfinite(input[index].x) && std::isfinite(input[index].y)) {
      output.push_back(input[index]);
    }
  }
  return output;
}

std::vector<ScanPoint2d> transformTarget(
  const std::vector<ScanPoint2d> & target, const Pose2d & source_to_target)
{
  const double cosine = std::cos(source_to_target.yaw);
  const double sine = std::sin(source_to_target.yaw);
  std::vector<ScanPoint2d> output;
  output.reserve(target.size());
  for (const auto & point : target) {
    output.push_back({
      cosine * point.x - sine * point.y + source_to_target.x,
      sine * point.x + cosine * point.y + source_to_target.y});
  }
  return output;
}

GridKey gridKey(const ScanPoint2d & point, const double cell_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / cell_size)),
    static_cast<std::int64_t>(std::floor(point.y / cell_size))};
}

SpatialGrid buildGrid(const std::vector<ScanPoint2d> & points, const double cell_size)
{
  SpatialGrid grid;
  for (const auto & point : points) {
    grid[gridKey(point, cell_size)].push_back(point);
  }
  return grid;
}

std::size_t countMatches(
  const std::vector<ScanPoint2d> & query, const SpatialGrid & reference,
  const double match_distance)
{
  const double maximum_distance_squared = match_distance * match_distance;
  std::size_t matches = 0U;
  for (const auto & point : query) {
    const auto center = gridKey(point, match_distance);
    bool matched = false;
    for (std::int64_t dx = -1; dx <= 1 && !matched; ++dx) {
      for (std::int64_t dy = -1; dy <= 1 && !matched; ++dy) {
        const auto cell = reference.find({center.x + dx, center.y + dy});
        if (cell == reference.end()) {
          continue;
        }
        for (const auto & candidate : cell->second) {
          const double difference_x = point.x - candidate.x;
          const double difference_y = point.y - candidate.y;
          if (difference_x * difference_x + difference_y * difference_y <
            maximum_distance_squared)
          {
            matched = true;
            break;
          }
        }
      }
    }
    matches += matched ? 1U : 0U;
  }
  return matches;
}

}  // namespace

ScanOverlapResult evaluateScanOverlap(
  const std::vector<ScanPoint2d> & source,
  const std::vector<ScanPoint2d> & target,
  const Pose2d & source_to_target,
  const ScanOverlapConfig & config)
{
  if (!(config.match_distance_m > 0.0) || !std::isfinite(config.match_distance_m) ||
    config.point_stride == 0U || config.minimum_points == 0U)
  {
    throw std::invalid_argument("scan-overlap configuration must be finite and positive");
  }

  ScanOverlapResult result;
  const auto sampled_source = sampleFinite(source, config.point_stride);
  const auto sampled_target = sampleFinite(target, config.point_stride);
  result.source_points = sampled_source.size();
  result.target_points = sampled_target.size();
  if (sampled_source.size() < config.minimum_points ||
    sampled_target.size() < config.minimum_points)
  {
    // 点数不足时 fail-open：缺测不能被当作“不重合”，否则玻璃/远距离稀疏扫描会被系统性误杀。
    return result;
  }

  const auto transformed_target = transformTarget(sampled_target, source_to_target);
  const auto source_grid = buildGrid(sampled_source, config.match_distance_m);
  const auto target_grid = buildGrid(transformed_target, config.match_distance_m);
  result.source_matches = countMatches(sampled_source, target_grid, config.match_distance_m);
  result.target_matches = countMatches(transformed_target, source_grid, config.match_distance_m);
  result.overlap_ratio = 0.5 * (
    static_cast<double>(result.source_matches) / static_cast<double>(sampled_source.size()) +
    static_cast<double>(result.target_matches) / static_cast<double>(sampled_target.size()));
  result.available = true;
  return result;
}

}  // namespace embodied_slam
