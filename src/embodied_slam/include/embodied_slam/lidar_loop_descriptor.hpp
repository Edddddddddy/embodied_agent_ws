#pragma once

#include <cstddef>
#include <vector>

namespace embodied_slam
{

struct LidarPoint2D
{
  double x{0.0};
  double y{0.0};
};

struct PolarDescriptorConfig
{
  std::size_t radial_bins{20};
  std::size_t angular_bins{60};
  double maximum_range_m{20.0};
  std::size_t minimum_points{30};
};

struct PolarScanDescriptor
{
  std::size_t radial_bins{0};
  std::size_t angular_bins{0};
  std::size_t valid_points{0};
  std::vector<double> cells;
  std::vector<double> ring_key;

  bool valid() const;
};

struct DescriptorAlignment
{
  double similarity{0.0};
  std::size_t sector_shift{0};
  double yaw_offset_rad{0.0};
};

// 将单帧 2D 激光投影为“径向环 × 方位扇区”占用描述子。它不读取位姿或真值，
// 因而可以作为回环候选生成的前端特征，而不是后端优化结果的别名。
PolarScanDescriptor makePolarScanDescriptor(
  const std::vector<LidarPoint2D> & points,
  const PolarDescriptorConfig & config = {});

// 穷举循环平移得到旋转不变相似度，同时返回供后续 scan matcher 初始化的偏航量。
DescriptorAlignment alignPolarDescriptors(
  const PolarScanDescriptor & query,
  const PolarScanDescriptor & candidate);

struct LoopCandidateIndexConfig
{
  double minimum_temporal_separation_s{60.0};
  std::size_t coarse_preselection_count{30};
  double minimum_similarity{0.0};
};

struct LidarLoopCandidate
{
  int scan_id{0};
  double stamp_s{0.0};
  double similarity{0.0};
  std::size_t sector_shift{0};
  double yaw_offset_rad{0.0};
  double ring_key_distance{0.0};
};

class LidarLoopCandidateIndex
{
public:
  explicit LidarLoopCandidateIndex(const LoopCandidateIndexConfig & config = {});

  void add(int scan_id, double stamp_s, const PolarScanDescriptor & descriptor);
  std::vector<LidarLoopCandidate> query(
    double stamp_s,
    const PolarScanDescriptor & descriptor,
    std::size_t top_k) const;
  std::size_t size() const;

private:
  struct Entry
  {
    int scan_id{0};
    double stamp_s{0.0};
    PolarScanDescriptor descriptor;
  };

  LoopCandidateIndexConfig config_;
  std::vector<Entry> entries_;
};

}  // namespace embodied_slam
