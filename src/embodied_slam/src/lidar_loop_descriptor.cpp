#include "embodied_slam/lidar_loop_descriptor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace embodied_slam
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

void validateConfig(const PolarDescriptorConfig & config)
{
  if (config.radial_bins == 0 || config.angular_bins < 4 ||
    !std::isfinite(config.maximum_range_m) || config.maximum_range_m <= 0.0 ||
    config.minimum_points == 0)
  {
    throw std::invalid_argument("invalid polar descriptor configuration");
  }
}

double ringKeyDistance(
  const PolarScanDescriptor & left,
  const PolarScanDescriptor & right)
{
  if (left.ring_key.size() != right.ring_key.size()) {
    throw std::invalid_argument("descriptor ring-key dimensions differ");
  }
  double squared_sum = 0.0;
  for (std::size_t index = 0; index < left.ring_key.size(); ++index) {
    const double delta = left.ring_key[index] - right.ring_key[index];
    squared_sum += delta * delta;
  }
  return std::sqrt(squared_sum);
}

void validateDescriptorPair(
  const PolarScanDescriptor & left,
  const PolarScanDescriptor & right)
{
  if (!left.valid() || !right.valid()) {
    throw std::invalid_argument("cannot compare invalid polar descriptors");
  }
  if (left.radial_bins != right.radial_bins || left.angular_bins != right.angular_bins ||
    left.cells.size() != right.cells.size())
  {
    throw std::invalid_argument("descriptor dimensions differ");
  }
}

}  // namespace

bool PolarScanDescriptor::valid() const
{
  return radial_bins > 0 && angular_bins > 0 && valid_points > 0 &&
         cells.size() == radial_bins * angular_bins && ring_key.size() == radial_bins;
}

PolarScanDescriptor makePolarScanDescriptor(
  const std::vector<LidarPoint2D> & points,
  const PolarDescriptorConfig & config)
{
  validateConfig(config);
  PolarScanDescriptor descriptor;
  descriptor.radial_bins = config.radial_bins;
  descriptor.angular_bins = config.angular_bins;
  descriptor.cells.assign(config.radial_bins * config.angular_bins, 0.0);
  descriptor.ring_key.assign(config.radial_bins, 0.0);

  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const double radius = std::hypot(point.x, point.y);
    if (radius <= 0.0 || radius >= config.maximum_range_m) {
      continue;
    }
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) {
      angle += kTwoPi;
    }
    const auto radial = std::min(
      config.radial_bins - 1,
      static_cast<std::size_t>(radius / config.maximum_range_m * config.radial_bins));
    const auto angular = std::min(
      config.angular_bins - 1,
      static_cast<std::size_t>(angle / kTwoPi * config.angular_bins));
    descriptor.cells[radial * config.angular_bins + angular] += 1.0;
    ++descriptor.valid_points;
  }

  if (descriptor.valid_points < config.minimum_points) {
    descriptor.valid_points = 0;
    return descriptor;
  }

  // log1p 抑制同一格内的高密度束数，避免某个近距离墙面支配整个余弦相似度。
  for (auto & value : descriptor.cells) {
    value = std::log1p(value);
  }
  for (std::size_t radial = 0; radial < config.radial_bins; ++radial) {
    double sum = 0.0;
    for (std::size_t angular = 0; angular < config.angular_bins; ++angular) {
      sum += descriptor.cells[radial * config.angular_bins + angular];
    }
    // 环键对方位旋转不敏感，只用于粗筛；完整描述子仍负责最终排序。
    descriptor.ring_key[radial] = sum / static_cast<double>(config.angular_bins);
  }
  return descriptor;
}

DescriptorAlignment alignPolarDescriptors(
  const PolarScanDescriptor & query,
  const PolarScanDescriptor & candidate)
{
  validateDescriptorPair(query, candidate);
  double query_norm_squared = 0.0;
  double candidate_norm_squared = 0.0;
  for (const double value : query.cells) {
    query_norm_squared += value * value;
  }
  for (const double value : candidate.cells) {
    candidate_norm_squared += value * value;
  }
  const double denominator = std::sqrt(query_norm_squared * candidate_norm_squared);
  if (denominator <= std::numeric_limits<double>::epsilon()) {
    return {};
  }

  DescriptorAlignment best;
  for (std::size_t shift = 0; shift < query.angular_bins; ++shift) {
    double dot = 0.0;
    for (std::size_t radial = 0; radial < query.radial_bins; ++radial) {
      for (std::size_t angular = 0; angular < query.angular_bins; ++angular) {
        const std::size_t candidate_angular = (angular + shift) % query.angular_bins;
        dot += query.cells[radial * query.angular_bins + angular] *
          candidate.cells[radial * query.angular_bins + candidate_angular];
      }
    }
    const double similarity = dot / denominator;
    if (similarity > best.similarity) {
      best.similarity = similarity;
      best.sector_shift = shift;
    }
  }
  best.yaw_offset_rad = kTwoPi * static_cast<double>(best.sector_shift) /
    static_cast<double>(query.angular_bins);
  return best;
}

LidarLoopCandidateIndex::LidarLoopCandidateIndex(const LoopCandidateIndexConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.minimum_temporal_separation_s) ||
    config_.minimum_temporal_separation_s < 0.0 ||
    config_.coarse_preselection_count == 0 ||
    !std::isfinite(config_.minimum_similarity) ||
    config_.minimum_similarity < 0.0 || config_.minimum_similarity > 1.0)
  {
    throw std::invalid_argument("invalid loop-candidate index configuration");
  }
}

void LidarLoopCandidateIndex::add(
  const int scan_id,
  const double stamp_s,
  const PolarScanDescriptor & descriptor)
{
  if (!descriptor.valid() || !std::isfinite(stamp_s)) {
    throw std::invalid_argument("candidate index requires a valid descriptor and timestamp");
  }
  const auto duplicate = std::find_if(
    entries_.begin(), entries_.end(),
    [scan_id](const Entry & entry) {return entry.scan_id == scan_id;});
  if (duplicate != entries_.end()) {
    throw std::invalid_argument("duplicate scan id in candidate index");
  }
  entries_.push_back({scan_id, stamp_s, descriptor});
}

std::vector<LidarLoopCandidate> LidarLoopCandidateIndex::query(
  const double stamp_s,
  const PolarScanDescriptor & descriptor,
  const std::size_t top_k) const
{
  if (!descriptor.valid() || !std::isfinite(stamp_s) || top_k == 0) {
    throw std::invalid_argument("query requires a valid descriptor, timestamp and top-k");
  }
  struct CoarseCandidate
  {
    const Entry * entry{nullptr};
    double distance{0.0};
  };
  std::vector<CoarseCandidate> coarse;
  for (const auto & entry : entries_) {
    // 禁止未来帧和时间上过近的帧进入候选池，否则连续扫描会制造虚假的高精度。
    const double separation = stamp_s - entry.stamp_s;
    if (separation + 1e-9 < config_.minimum_temporal_separation_s) {
      continue;
    }
    validateDescriptorPair(descriptor, entry.descriptor);
    coarse.push_back({&entry, ringKeyDistance(descriptor, entry.descriptor)});
  }
  std::sort(
    coarse.begin(), coarse.end(), [](const auto & left, const auto & right) {
      if (left.distance != right.distance) {
        return left.distance < right.distance;
      }
      return left.entry->scan_id < right.entry->scan_id;
    });
  const std::size_t preselection = std::min(
    coarse.size(), std::max(top_k, config_.coarse_preselection_count));

  std::vector<LidarLoopCandidate> result;
  result.reserve(preselection);
  for (std::size_t index = 0; index < preselection; ++index) {
    const auto alignment = alignPolarDescriptors(descriptor, coarse[index].entry->descriptor);
    if (alignment.similarity + 1e-12 < config_.minimum_similarity) {
      continue;
    }
    result.push_back(
      {coarse[index].entry->scan_id, coarse[index].entry->stamp_s,
        alignment.similarity, alignment.sector_shift, alignment.yaw_offset_rad,
        coarse[index].distance});
  }
  std::sort(
    result.begin(), result.end(), [](const auto & left, const auto & right) {
      if (left.ring_key_distance != right.ring_key_distance) {
        return left.ring_key_distance < right.ring_key_distance;
      }
      // 真实走廊数据表明，平移视角变化会让稀疏占用矩阵的全局余弦重排退化；
      // 因此环键负责检索排序，完整描述子只在同环键距离时破平并提供偏航初值。
      if (left.similarity != right.similarity) {
        return left.similarity > right.similarity;
      }
      return left.scan_id < right.scan_id;
    });
  if (result.size() > top_k) {
    result.resize(top_k);
  }
  return result;
}

std::size_t LidarLoopCandidateIndex::size() const
{
  return entries_.size();
}

}  // namespace embodied_slam
