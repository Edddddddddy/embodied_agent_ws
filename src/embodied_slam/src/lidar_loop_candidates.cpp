#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "embodied_slam/lidar_loop_descriptor.hpp"

namespace
{

struct ScanRow
{
  int scan_id{0};
  double stamp_s{0.0};
  std::vector<embodied_slam::LidarPoint2D> points;
};

std::vector<ScanRow> loadCorpus(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open scan corpus: " + path);
  }
  std::vector<ScanRow> rows;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream stream(line);
    char kind = '\0';
    std::size_t point_count = 0;
    ScanRow row;
    if (!(stream >> kind >> row.scan_id >> row.stamp_s >> point_count) || kind != 'S') {
      throw std::runtime_error("invalid corpus row at line " + std::to_string(line_number));
    }
    row.points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      embodied_slam::LidarPoint2D point;
      if (!(stream >> point.x >> point.y)) {
        throw std::runtime_error(
                "truncated point list at line " + std::to_string(line_number));
      }
      row.points.push_back(point);
    }
    std::string trailing;
    if (stream >> trailing) {
      throw std::runtime_error("unexpected corpus columns at line " + std::to_string(line_number));
    }
    if (!rows.empty() && row.stamp_s < rows.back().stamp_s) {
      throw std::runtime_error("scan corpus timestamps must be monotonic");
    }
    rows.push_back(std::move(row));
  }
  if (rows.empty()) {
    throw std::runtime_error("scan corpus is empty");
  }
  return rows;
}

void usage(const char * program)
{
  std::cerr << "Usage: " << program
            << " INPUT OUTPUT [radial_bins angular_bins max_range min_points"
            << " min_separation_s preselection top_k min_similarity]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 11) {
    usage(argv[0]);
    return 2;
  }
  try {
    embodied_slam::PolarDescriptorConfig descriptor_config;
    embodied_slam::LoopCandidateIndexConfig index_config;
    std::size_t top_k = 10;
    if (argc > 3) {descriptor_config.radial_bins = std::stoul(argv[3]);}
    if (argc > 4) {descriptor_config.angular_bins = std::stoul(argv[4]);}
    if (argc > 5) {descriptor_config.maximum_range_m = std::stod(argv[5]);}
    if (argc > 6) {descriptor_config.minimum_points = std::stoul(argv[6]);}
    if (argc > 7) {index_config.minimum_temporal_separation_s = std::stod(argv[7]);}
    if (argc > 8) {index_config.coarse_preselection_count = std::stoul(argv[8]);}
    if (argc > 9) {top_k = std::stoul(argv[9]);}
    if (argc > 10) {index_config.minimum_similarity = std::stod(argv[10]);}

    const auto rows = loadCorpus(argv[1]);
    std::ofstream output(argv[2], std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open candidate output: " + std::string(argv[2]));
    }
    output << std::setprecision(12);

    embodied_slam::LidarLoopCandidateIndex index(index_config);
    std::size_t valid_descriptors = 0;
    std::size_t emitted_candidates = 0;
    for (const auto & row : rows) {
      const auto descriptor = embodied_slam::makePolarScanDescriptor(
        row.points, descriptor_config);
      output << "{\"schema_version\":1,\"query_id\":" << row.scan_id
             << ",\"query_stamp_s\":" << row.stamp_s
             << ",\"database_size\":" << index.size()
             << ",\"descriptor_valid\":" << (descriptor.valid() ? "true" : "false")
             << ",\"valid_points\":" << descriptor.valid_points
             << ",\"candidates\":[";
      std::vector<embodied_slam::LidarLoopCandidate> candidates;
      if (descriptor.valid()) {
        candidates = index.query(row.stamp_s, descriptor, top_k);
      }
      for (std::size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
        const auto & candidate = candidates[candidate_index];
        if (candidate_index > 0) {
          output << ',';
        }
        output << "{\"rank\":" << candidate_index + 1
               << ",\"scan_id\":" << candidate.scan_id
               << ",\"stamp_s\":" << candidate.stamp_s
               << ",\"similarity\":" << candidate.similarity
               << ",\"sector_shift\":" << candidate.sector_shift
               << ",\"yaw_offset_rad\":" << candidate.yaw_offset_rad
               << ",\"ring_key_distance\":" << candidate.ring_key_distance << '}';
      }
      output << "]}\n";
      emitted_candidates += candidates.size();
      if (descriptor.valid()) {
        ++valid_descriptors;
        index.add(row.scan_id, row.stamp_s, descriptor);
      }
    }

    std::cout << std::setprecision(12)
              << "{\"schema_version\":1,\"scan_rows\":" << rows.size()
              << ",\"valid_descriptors\":" << valid_descriptors
              << ",\"emitted_candidates\":" << emitted_candidates
              << ",\"top_k\":" << top_k
              << ",\"minimum_temporal_separation_s\":"
              << index_config.minimum_temporal_separation_s
              << ",\"coarse_preselection_count\":"
              << index_config.coarse_preselection_count
              << ",\"minimum_similarity\":" << index_config.minimum_similarity
              << "}\n";
    return valid_descriptors == rows.size() ? 0 : 1;
  } catch (const std::exception & exception) {
    std::cerr << "lidar_loop_candidates: " << exception.what() << '\n';
    return 2;
  }
}
