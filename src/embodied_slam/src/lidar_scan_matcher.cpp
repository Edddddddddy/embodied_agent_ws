#include "embodied_slam/lidar_scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "embodied_slam/scan_overlap_validator.hpp"

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

using SpatialGrid = std::unordered_map<GridKey, std::vector<LidarPoint2D>, GridKeyHash>;

struct Correspondence
{
  LidarPoint2D moving;
  LidarPoint2D fixed;
  double squared_distance{0.0};
};

struct IterationResult
{
  Pose2d pose;
  bool available{false};
  bool converged{false};
  std::size_t iterations{0U};
};

void validateConfig(const LidarScanMatcherConfig & config)
{
  const bool fractions_valid = std::isfinite(config.trim_fraction) &&
    config.trim_fraction > 0.0 && config.trim_fraction <= 1.0;
  if (config.point_stride == 0U || config.minimum_points < 3U ||
    config.coarse_iterations == 0U || config.fine_iterations == 0U ||
    !std::isfinite(config.coarse_max_correspondence_m) ||
    !std::isfinite(config.fine_max_correspondence_m) ||
    config.coarse_max_correspondence_m <= config.fine_max_correspondence_m ||
    config.fine_max_correspondence_m <= 0.0 || !fractions_valid ||
    !std::isfinite(config.translation_tolerance_m) || config.translation_tolerance_m <= 0.0 ||
    !std::isfinite(config.rotation_tolerance_rad) || config.rotation_tolerance_rad <= 0.0 ||
    !std::isfinite(config.maximum_translation_m) || config.maximum_translation_m <= 0.0 ||
    !std::isfinite(config.minimum_inlier_ratio) || config.minimum_inlier_ratio <= 0.0 ||
    config.minimum_inlier_ratio > 1.0 || !std::isfinite(config.maximum_rmse_m) ||
    config.maximum_rmse_m <= 0.0 || !std::isfinite(config.minimum_observability_ratio) ||
    config.minimum_observability_ratio < 0.0 || config.minimum_observability_ratio > 1.0 ||
    !std::isfinite(config.ambiguity_overlap_tolerance) ||
    config.ambiguity_overlap_tolerance < 0.0 ||
    !std::isfinite(config.ambiguity_rmse_tolerance_m) ||
    config.ambiguity_rmse_tolerance_m < 0.0 ||
    !std::isfinite(config.minimum_ambiguous_yaw_separation_rad) ||
    config.minimum_ambiguous_yaw_separation_rad <= 0.0 ||
    !std::isfinite(config.maximum_prior_yaw_deviation_rad) ||
    config.maximum_prior_yaw_deviation_rad <= 0.0 ||
    !std::isfinite(config.maximum_prior_translation_deviation_m) ||
    config.maximum_prior_translation_deviation_m <= 0.0)
  {
    throw std::invalid_argument("invalid lidar scan-matcher configuration");
  }
}

std::vector<LidarPoint2D> sampleFinite(
  const std::vector<LidarPoint2D> & input, const std::size_t stride)
{
  std::vector<LidarPoint2D> output;
  output.reserve((input.size() + stride - 1U) / stride);
  for (std::size_t index = 0; index < input.size(); index += stride) {
    if (std::isfinite(input[index].x) && std::isfinite(input[index].y)) {
      output.push_back(input[index]);
    }
  }
  return output;
}

LidarPoint2D transformPoint(const LidarPoint2D & point, const Pose2d & pose)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  return {
    cosine * point.x - sine * point.y + pose.x,
    sine * point.x + cosine * point.y + pose.y};
}

GridKey gridKey(const LidarPoint2D & point, const double cell_size)
{
  return {
    static_cast<std::int64_t>(std::floor(point.x / cell_size)),
    static_cast<std::int64_t>(std::floor(point.y / cell_size))};
}

SpatialGrid buildGrid(const std::vector<LidarPoint2D> & points, const double cell_size)
{
  SpatialGrid grid;
  for (const auto & point : points) {
    grid[gridKey(point, cell_size)].push_back(point);
  }
  return grid;
}

std::vector<Correspondence> findCorrespondences(
  const SpatialGrid & source_grid,
  const std::vector<LidarPoint2D> & target,
  const Pose2d & target_to_source,
  const double maximum_distance)
{
  const double maximum_squared = maximum_distance * maximum_distance;
  std::vector<Correspondence> correspondences;
  correspondences.reserve(target.size());
  for (const auto & target_point : target) {
    const auto moving = transformPoint(target_point, target_to_source);
    const auto center = gridKey(moving, maximum_distance);
    double best_squared = maximum_squared;
    LidarPoint2D best;
    bool found = false;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        const auto cell = source_grid.find({center.x + dx, center.y + dy});
        if (cell == source_grid.end()) {
          continue;
        }
        for (const auto & candidate : cell->second) {
          const double difference_x = moving.x - candidate.x;
          const double difference_y = moving.y - candidate.y;
          const double squared = difference_x * difference_x + difference_y * difference_y;
          if (squared <= best_squared) {
            best_squared = squared;
            best = candidate;
            found = true;
          }
        }
      }
    }
    if (found) {
      correspondences.push_back({moving, best, best_squared});
    }
  }
  return correspondences;
}

Pose2d estimateIncrement(const std::vector<Correspondence> & correspondences)
{
  LidarPoint2D moving_centroid;
  LidarPoint2D fixed_centroid;
  for (const auto & correspondence : correspondences) {
    moving_centroid.x += correspondence.moving.x;
    moving_centroid.y += correspondence.moving.y;
    fixed_centroid.x += correspondence.fixed.x;
    fixed_centroid.y += correspondence.fixed.y;
  }
  const double count = static_cast<double>(correspondences.size());
  moving_centroid.x /= count;
  moving_centroid.y /= count;
  fixed_centroid.x /= count;
  fixed_centroid.y /= count;

  double dot = 0.0;
  double cross = 0.0;
  for (const auto & correspondence : correspondences) {
    const double moving_x = correspondence.moving.x - moving_centroid.x;
    const double moving_y = correspondence.moving.y - moving_centroid.y;
    const double fixed_x = correspondence.fixed.x - fixed_centroid.x;
    const double fixed_y = correspondence.fixed.y - fixed_centroid.y;
    dot += moving_x * fixed_x + moving_y * fixed_y;
    cross += moving_x * fixed_y - moving_y * fixed_x;
  }
  const double yaw = std::atan2(cross, dot);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    fixed_centroid.x - (cosine * moving_centroid.x - sine * moving_centroid.y),
    fixed_centroid.y - (sine * moving_centroid.x + cosine * moving_centroid.y),
    yaw};
}

Pose2d compose(const Pose2d & left, const Pose2d & right)
{
  const double cosine = std::cos(left.yaw);
  const double sine = std::sin(left.yaw);
  return {
    cosine * right.x - sine * right.y + left.x,
    sine * right.x + cosine * right.y + left.y,
    normalize_angle(left.yaw + right.yaw)};
}

Pose2d centroidSeed(
  const std::vector<LidarPoint2D> & source,
  const std::vector<LidarPoint2D> & target,
  const double yaw)
{
  LidarPoint2D source_centroid;
  LidarPoint2D target_centroid;
  for (const auto & point : source) {
    source_centroid.x += point.x;
    source_centroid.y += point.y;
  }
  for (const auto & point : target) {
    target_centroid.x += point.x;
    target_centroid.y += point.y;
  }
  source_centroid.x /= static_cast<double>(source.size());
  source_centroid.y /= static_cast<double>(source.size());
  target_centroid.x /= static_cast<double>(target.size());
  target_centroid.y /= static_cast<double>(target.size());
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return {
    source_centroid.x - (cosine * target_centroid.x - sine * target_centroid.y),
    source_centroid.y - (sine * target_centroid.x + cosine * target_centroid.y),
    normalize_angle(yaw)};
}

IterationResult iterateIcp(
  const std::vector<LidarPoint2D> & source,
  const std::vector<LidarPoint2D> & target,
  const Pose2d & initial,
  const double maximum_distance,
  const std::size_t maximum_iterations,
  const LidarScanMatcherConfig & config)
{
  const auto grid = buildGrid(source, maximum_distance);
  Pose2d current = initial;
  IterationResult result;
  result.pose = current;
  for (std::size_t iteration = 0; iteration < maximum_iterations; ++iteration) {
    auto correspondences = findCorrespondences(grid, target, current, maximum_distance);
    if (correspondences.size() < config.minimum_points) {
      return result;
    }
    std::sort(
      correspondences.begin(), correspondences.end(),
      [](const auto & left, const auto & right) {
        return left.squared_distance < right.squared_distance;
      });
    const std::size_t trimmed_count = std::max(
      config.minimum_points,
      static_cast<std::size_t>(std::ceil(config.trim_fraction * correspondences.size())));
    correspondences.resize(std::min(trimmed_count, correspondences.size()));
    const Pose2d increment = estimateIncrement(correspondences);
    current = compose(increment, current);
    result.available = true;
    result.pose = current;
    result.iterations = iteration + 1U;
    if (std::hypot(increment.x, increment.y) <= config.translation_tolerance_m &&
      std::abs(increment.yaw) <= config.rotation_tolerance_rad)
    {
      result.converged = true;
      break;
    }
  }
  return result;
}

double observabilityRatio(const std::vector<Correspondence> & correspondences)
{
  if (correspondences.size() < 3U) {
    return 0.0;
  }
  double center_x = 0.0;
  double center_y = 0.0;
  for (const auto & correspondence : correspondences) {
    center_x += correspondence.fixed.x;
    center_y += correspondence.fixed.y;
  }
  center_x /= static_cast<double>(correspondences.size());
  center_y /= static_cast<double>(correspondences.size());
  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (const auto & correspondence : correspondences) {
    const double x = correspondence.fixed.x - center_x;
    const double y = correspondence.fixed.y - center_y;
    xx += x * x;
    xy += x * y;
    yy += y * y;
  }
  const double trace = xx + yy;
  const double discriminant = std::sqrt(
    std::max(0.0, (xx - yy) * (xx - yy) + 4.0 * xy * xy));
  const double maximum = 0.5 * (trace + discriminant);
  const double minimum = 0.5 * (trace - discriminant);
  return maximum <= std::numeric_limits<double>::epsilon() ? 0.0 : minimum / maximum;
}

double correspondenceRmse(const std::vector<Correspondence> & correspondences)
{
  if (correspondences.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (const auto & correspondence : correspondences) {
    sum += correspondence.squared_distance;
  }
  return std::sqrt(sum / static_cast<double>(correspondences.size()));
}

LidarScanMatchResult runSeed(
  const std::vector<LidarPoint2D> & source,
  const std::vector<LidarPoint2D> & target,
  const double descriptor_yaw,
  const Pose2d & initial,
  const LidarScanMatcherConfig & config)
{
  LidarScanMatchResult result;
  result.source_points = source.size();
  result.target_points = target.size();
  result.descriptor_yaw_offset_rad = descriptor_yaw;
  result.selected_initial_yaw_rad = initial.yaw;
  const auto coarse = iterateIcp(
    source, target, initial,
    config.coarse_max_correspondence_m, config.coarse_iterations, config);
  if (!coarse.available) {
    result.rejection_reason = "insufficient_coarse_correspondences";
    return result;
  }
  const auto fine = iterateIcp(
    source, target, coarse.pose,
    config.fine_max_correspondence_m, config.fine_iterations, config);
  result.available = fine.available;
  result.converged = fine.converged;
  result.target_to_source = fine.pose;
  result.iterations = coarse.iterations + fine.iterations;
  if (!fine.available) {
    result.rejection_reason = "insufficient_fine_correspondences";
    return result;
  }

  const auto fine_grid = buildGrid(source, config.fine_max_correspondence_m);
  const auto correspondences = findCorrespondences(
    fine_grid, target, fine.pose, config.fine_max_correspondence_m);
  result.correspondences = correspondences.size();
  result.inlier_ratio = static_cast<double>(correspondences.size()) /
    static_cast<double>(target.size());
  result.rmse_m = correspondenceRmse(correspondences);
  result.observability_ratio = observabilityRatio(correspondences);

  std::vector<ScanPoint2d> overlap_source;
  std::vector<ScanPoint2d> overlap_target;
  overlap_source.reserve(source.size());
  overlap_target.reserve(target.size());
  for (const auto & point : source) {overlap_source.push_back({point.x, point.y});}
  for (const auto & point : target) {overlap_target.push_back({point.x, point.y});}
  const auto overlap = evaluateScanOverlap(
    overlap_source, overlap_target, fine.pose,
    {config.fine_max_correspondence_m, 1U, config.minimum_points});
  result.bidirectional_overlap_ratio = overlap.available ? overlap.overlap_ratio : 0.0;

  if (!result.converged) {
    result.rejection_reason = "not_converged";
  } else if (std::hypot(fine.pose.x, fine.pose.y) > config.maximum_translation_m) {
    result.rejection_reason = "translation_limit";
  } else if (result.inlier_ratio < config.minimum_inlier_ratio) {
    result.rejection_reason = "low_inlier_ratio";
  } else if (result.rmse_m > config.maximum_rmse_m) {
    result.rejection_reason = "high_rmse";
  } else if (result.observability_ratio < config.minimum_observability_ratio) {
    result.rejection_reason = "degenerate_geometry";
  } else {
    result.accepted = true;
    result.rejection_reason = "accepted";
  }
  return result;
}

bool betterResult(const LidarScanMatchResult & left, const LidarScanMatchResult & right)
{
  if (left.available != right.available) {
    return left.available;
  }
  if (left.converged != right.converged) {
    return left.converged;
  }
  // 多初值可能在重复走廊中收敛到高重叠但超出安全平移范围的别处。
  // 先保留通过运行时安全门的局部解，再在同等级解之间比较重叠与 RMSE。
  if (left.accepted != right.accepted) {
    return left.accepted;
  }
  if (left.bidirectional_overlap_ratio != right.bidirectional_overlap_ratio) {
    return left.bidirectional_overlap_ratio > right.bidirectional_overlap_ratio;
  }
  return left.rmse_m < right.rmse_m;
}

void addUniqueSeed(std::vector<double> & seeds, const double seed)
{
  const double normalized = normalize_angle(seed);
  const auto duplicate = std::find_if(
    seeds.begin(), seeds.end(), [normalized](const double existing) {
      return std::abs(normalize_angle(existing - normalized)) <= 1e-9;
    });
  if (duplicate == seeds.end()) {
    seeds.push_back(normalized);
  }
}

}  // namespace

LidarScanMatchResult matchLidarScans(
  const std::vector<LidarPoint2D> & source_input,
  const std::vector<LidarPoint2D> & target_input,
  const double descriptor_yaw_offset_rad,
  const LidarScanMatcherConfig & config,
  const std::optional<Pose2d> odometry_prior)
{
  validateConfig(config);
  if (!std::isfinite(descriptor_yaw_offset_rad)) {
    throw std::invalid_argument("descriptor yaw offset must be finite");
  }
  if (odometry_prior &&
    (!std::isfinite(odometry_prior->x) || !std::isfinite(odometry_prior->y) ||
    !std::isfinite(odometry_prior->yaw)))
  {
    throw std::invalid_argument("odometry prior must be finite");
  }
  const auto source = sampleFinite(source_input, config.point_stride);
  const auto target = sampleFinite(target_input, config.point_stride);
  if (source.size() < config.minimum_points || target.size() < config.minimum_points) {
    LidarScanMatchResult result;
    result.source_points = source.size();
    result.target_points = target.size();
    result.descriptor_yaw_offset_rad = descriptor_yaw_offset_rad;
    result.rejection_reason = "insufficient_points";
    return result;
  }

  // 走廊和矩形房间常出现 0/180 度对称。除循环移位的正负号外，再显式验证
  // 半周对称种子；若两个远离的姿态得到近似同分几何解，则宁可拒绝也不造假回环。
  constexpr double kPi = 3.14159265358979323846;
  std::vector<double> seeds;
  addUniqueSeed(seeds, descriptor_yaw_offset_rad);
  addUniqueSeed(seeds, -descriptor_yaw_offset_rad);
  addUniqueSeed(seeds, descriptor_yaw_offset_rad + kPi);
  addUniqueSeed(seeds, -descriptor_yaw_offset_rad + kPi);
  std::vector<LidarScanMatchResult> results;
  results.reserve(seeds.size() * 2U + (odometry_prior ? 1U : 0U));
  for (const double seed : seeds) {
    // 回环候选由“空间上应接近”的检索层产生，零平移初值能避免长走廊的
    // 点云质心把 ICP 拉到数米外的重复墙段；质心初值仍保留用于较大视角变化。
    results.push_back(runSeed(
        source, target, descriptor_yaw_offset_rad, Pose2d{0.0, 0.0, seed}, config));
    results.push_back(runSeed(
        source, target, descriptor_yaw_offset_rad,
        centroidSeed(source, target, seed), config));
  }
  if (odometry_prior) {
    Pose2d normalized_prior = *odometry_prior;
    normalized_prior.yaw = normalize_angle(normalized_prior.yaw);
    results.push_back(runSeed(
        source, target, descriptor_yaw_offset_rad, normalized_prior, config));
  }
  auto best_iterator = results.begin();
  if (odometry_prior) {
    auto prior_best = results.end();
    for (auto iterator = results.begin(); iterator != results.end(); ++iterator) {
      if (!iterator->available || !iterator->converged) {
        continue;
      }
      const double yaw_error = std::abs(normalize_angle(
          iterator->target_to_source.yaw - odometry_prior->yaw));
      const double translation_error = std::hypot(
        iterator->target_to_source.x - odometry_prior->x,
        iterator->target_to_source.y - odometry_prior->y);
      if (yaw_error > config.maximum_prior_yaw_deviation_rad ||
        (config.gate_prior_translation &&
        translation_error > config.maximum_prior_translation_deviation_m))
      {
        continue;
      }
      if (prior_best == results.end() || betterResult(*iterator, *prior_best)) {
        prior_best = iterator;
      }
    }
    if (prior_best != results.end()) {
      best_iterator = prior_best;
    } else {
      best_iterator = std::max_element(
        results.begin(), results.end(), [](const auto & left, const auto & right) {
          return betterResult(right, left);
        });
    }
  } else {
    best_iterator = std::max_element(
      results.begin(), results.end(), [](const auto & left, const auto & right) {
        return betterResult(right, left);
      });
  }
  auto best = *best_iterator;
  if (odometry_prior) {
    best.odometry_prior_used = true;
    best.odometry_prior_yaw_error_rad = std::abs(normalize_angle(
        best.target_to_source.yaw - odometry_prior->yaw));
    best.odometry_prior_translation_error_m = std::hypot(
      best.target_to_source.x - odometry_prior->x,
      best.target_to_source.y - odometry_prior->y);
    best.odometry_prior_consistent =
      best.odometry_prior_yaw_error_rad <= config.maximum_prior_yaw_deviation_rad &&
      (!config.gate_prior_translation ||
      best.odometry_prior_translation_error_m <=
      config.maximum_prior_translation_deviation_m);
    if (!best.odometry_prior_consistent) {
      best.accepted = false;
      best.rejection_reason = "odometry_prior_inconsistent";
      return best;
    }
    // 里程计只解除近似同分的半周对称，不参与 ICP 代价，也不替代几何门限。
    return best;
  }
  for (const auto & alternative : results) {
    if (&alternative == &(*best_iterator) || !alternative.available || !alternative.converged) {
      continue;
    }
    const double yaw_separation = std::abs(normalize_angle(
        alternative.target_to_source.yaw - best.target_to_source.yaw));
    const bool near_equal_geometry =
      alternative.bidirectional_overlap_ratio + config.ambiguity_overlap_tolerance >=
      best.bidirectional_overlap_ratio &&
      alternative.rmse_m <= best.rmse_m + config.ambiguity_rmse_tolerance_m;
    if (yaw_separation >= config.minimum_ambiguous_yaw_separation_rad &&
      near_equal_geometry)
    {
      best.yaw_ambiguous = true;
      best.accepted = false;
      best.rejection_reason = "yaw_ambiguous";
      best.alternate_overlap_ratio = alternative.bidirectional_overlap_ratio;
      best.alternate_yaw_separation_rad = yaw_separation;
      break;
    }
  }
  return best;
}

}  // namespace embodied_slam
