#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "embodied_slam/lidar_scan_corpus.hpp"
#include "embodied_slam/lidar_scan_matcher.hpp"
#include "embodied_slam/lidar_submap_builder.hpp"

namespace
{

struct CandidatePair
{
  int query_id{0};
  int candidate_id{0};
  double query_stamp_s{0.0};
  double candidate_stamp_s{0.0};
  std::size_t rank{0U};
  double yaw_offset_rad{0.0};
  double similarity{0.0};
  double ring_key_distance{0.0};
  std::optional<embodied_slam::Pose2d> odometry_prior;
};

std::vector<CandidatePair> loadPairs(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open candidate pairs: " + path);
  }
  std::vector<CandidatePair> pairs;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream stream(line);
    char kind = '\0';
    CandidatePair pair;
    if (!(stream >> kind >> pair.query_id >> pair.candidate_id >> pair.query_stamp_s >>
      pair.candidate_stamp_s >> pair.rank >> pair.yaw_offset_rad >> pair.similarity >>
      pair.ring_key_distance) || kind != 'P')
    {
      throw std::runtime_error("invalid candidate pair at line " + std::to_string(line_number));
    }
    double prior_x = 0.0;
    if (stream >> prior_x) {
      embodied_slam::Pose2d prior;
      prior.x = prior_x;
      if (!(stream >> prior.y >> prior.yaw)) {
        throw std::runtime_error("truncated odometry prior at line " + std::to_string(line_number));
      }
      pair.odometry_prior = prior;
    }
    std::string trailing;
    if (stream >> trailing) {
      throw std::runtime_error("unexpected pair columns at line " + std::to_string(line_number));
    }
    pairs.push_back(pair);
  }
  if (pairs.empty()) {
    throw std::runtime_error("candidate pair file is empty");
  }
  return pairs;
}

void writeBoolean(std::ostream & output, const bool value)
{
  output << (value ? "true" : "false");
}

void usage(const char * program)
{
  std::cerr << "Usage: " << program
            << " SCAN_CORPUS PAIR_FILE OUTPUT [point_stride minimum_points] "
            << "[--point-stride N --minimum-points N --submap-odometry PATH "
            << "--submap-half-window N --submap-max-time-delta S "
            << "--submap-point-stride N --submap-minimum-scans N "
            << "--submap-minimum-points N]" << '\n';
}

struct CliOptions
{
  embodied_slam::LidarScanMatcherConfig matcher;
  embodied_slam::LidarSubmapConfig submap;
  std::optional<std::string> odometry_path;
};

CliOptions parseOptions(const int argc, char ** argv)
{
  CliOptions options;
  bool submap_option_seen = false;
  // 保留既有 CTest/脚本的两个位置参数；新增模式一律使用显式 flag，避免参数串位。
  if (argc == 6 && argv[4][0] != '-') {
    options.matcher.point_stride = std::stoul(argv[4]);
    options.matcher.minimum_points = std::stoul(argv[5]);
    return options;
  }
  for (int index = 4; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--point-stride") {
      options.matcher.point_stride = std::stoul(value);
    } else if (argument == "--minimum-points") {
      options.matcher.minimum_points = std::stoul(value);
    } else if (argument == "--submap-odometry") {
      options.odometry_path = value;
      submap_option_seen = true;
    } else if (argument == "--submap-half-window") {
      options.submap.half_window_scans = std::stoul(value);
      submap_option_seen = true;
    } else if (argument == "--submap-max-time-delta") {
      options.submap.maximum_time_delta_s = std::stod(value);
      submap_option_seen = true;
    } else if (argument == "--submap-point-stride") {
      options.submap.point_stride = std::stoul(value);
      submap_option_seen = true;
    } else if (argument == "--submap-minimum-scans") {
      options.submap.minimum_contributing_scans = std::stoul(value);
      submap_option_seen = true;
    } else if (argument == "--submap-minimum-points") {
      options.submap.minimum_points = std::stoul(value);
      submap_option_seen = true;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (submap_option_seen && !options.odometry_path) {
    throw std::invalid_argument("submap options require --submap-odometry");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 4) {
    usage(argv[0]);
    return 2;
  }
  try {
    const auto options = parseOptions(argc, argv);

    const auto scans = embodied_slam::loadLidarScanCorpus(argv[1]);
    std::unordered_map<int, const embodied_slam::LidarScanRecord *> scan_by_id;
    for (const auto & scan : scans) {
      if (!scan_by_id.emplace(scan.scan_id, &scan).second) {
        throw std::runtime_error("duplicate scan id in corpus");
      }
    }
    std::unique_ptr<embodied_slam::LidarSubmapBuilder> submap_builder;
    std::unordered_map<int, embodied_slam::LidarSubmap> submap_cache;
    if (options.odometry_path) {
      const auto odometry = embodied_slam::loadLidarOdometryCorpus(*options.odometry_path);
      submap_builder = std::make_unique<embodied_slam::LidarSubmapBuilder>(
        scans, odometry, options.submap);
    }
    const auto pairs = loadPairs(argv[2]);
    std::ofstream output(argv[3], std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("cannot open shadow output: " + std::string(argv[3]));
    }
    output << std::setprecision(12);

    std::size_t available = 0U;
    std::size_t accepted = 0U;
    for (const auto & pair : pairs) {
      const auto query = scan_by_id.find(pair.query_id);
      const auto candidate = scan_by_id.find(pair.candidate_id);
      if (query == scan_by_id.end() || candidate == scan_by_id.end()) {
        throw std::runtime_error("candidate pair references missing scan id");
      }
      const std::vector<embodied_slam::LidarPoint2D> * query_points = &query->second->points;
      const std::vector<embodied_slam::LidarPoint2D> * candidate_points =
        &candidate->second->points;
      std::size_t query_submap_scans = 1U;
      std::size_t candidate_submap_scans = 1U;
      embodied_slam::LidarScanMatchResult result;
      if (submap_builder) {
        auto get_submap = [&](const int scan_id) -> const embodied_slam::LidarSubmap & {
            const auto existing = submap_cache.find(scan_id);
            if (existing != submap_cache.end()) {
              return existing->second;
            }
            return submap_cache.emplace(scan_id, submap_builder->build(scan_id)).first->second;
          };
        const auto & query_submap = get_submap(pair.query_id);
        const auto & candidate_submap = get_submap(pair.candidate_id);
        query_submap_scans = query_submap.contributing_scan_ids.size();
        candidate_submap_scans = candidate_submap.contributing_scan_ids.size();
        if (query_submap.available && candidate_submap.available) {
          query_points = &query_submap.points;
          candidate_points = &candidate_submap.points;
        } else {
          result.rejection_reason = query_submap.available ?
            "candidate_submap_unavailable" : "query_submap_unavailable";
        }
      }
      if (result.rejection_reason.empty()) {
        result = embodied_slam::matchLidarScans(
          *query_points, *candidate_points, pair.yaw_offset_rad, options.matcher,
          pair.odometry_prior);
      }
      available += result.available ? 1U : 0U;
      accepted += result.accepted ? 1U : 0U;
      output << "{\"schema_version\":1,\"query_id\":" << pair.query_id
             << ",\"candidate_id\":" << pair.candidate_id
             << ",\"query_stamp_s\":" << pair.query_stamp_s
             << ",\"candidate_stamp_s\":" << pair.candidate_stamp_s
             << ",\"candidate_rank\":" << pair.rank
             << ",\"similarity\":" << pair.similarity
             << ",\"ring_key_distance\":" << pair.ring_key_distance
             << ",\"matching_mode\":\""
             << (submap_builder ? "scan_to_submap" : "scan_to_scan") << '\"'
             << ",\"matcher_point_stride\":" << options.matcher.point_stride
             << ",\"submap_point_stride\":"
             << (submap_builder ? options.submap.point_stride : 1U)
             << ",\"effective_point_stride\":"
             << options.matcher.point_stride *
        (submap_builder ? options.submap.point_stride : 1U)
             << ",\"matcher_minimum_points\":" << options.matcher.minimum_points
             << ",\"query_submap_scans\":" << query_submap_scans
             << ",\"candidate_submap_scans\":" << candidate_submap_scans
             << ",\"query_geometry_points\":" << query_points->size()
             << ",\"candidate_geometry_points\":" << candidate_points->size()
             << ",\"descriptor_yaw_offset_rad\":" << pair.yaw_offset_rad
             << ",\"selected_initial_yaw_rad\":" << result.selected_initial_yaw_rad
             << ",\"available\":";
      writeBoolean(output, result.available);
      output << ",\"converged\":";
      writeBoolean(output, result.converged);
      output << ",\"accepted\":";
      writeBoolean(output, result.accepted);
      output << ",\"yaw_ambiguous\":";
      writeBoolean(output, result.yaw_ambiguous);
      output << ",\"odometry_prior_used\":";
      writeBoolean(output, result.odometry_prior_used);
      output << ",\"odometry_prior_consistent\":";
      writeBoolean(output, result.odometry_prior_consistent);
      output << ",\"target_to_source\":{\"x_m\":" << result.target_to_source.x
             << ",\"y_m\":" << result.target_to_source.y
             << ",\"yaw_rad\":" << result.target_to_source.yaw << '}'
             << ",\"iterations\":" << result.iterations
             << ",\"source_points\":" << result.source_points
             << ",\"target_points\":" << result.target_points
             << ",\"correspondences\":" << result.correspondences
             << ",\"inlier_ratio\":" << result.inlier_ratio
             << ",\"bidirectional_overlap_ratio\":"
             << result.bidirectional_overlap_ratio
             << ",\"rmse_m\":" << result.rmse_m
             << ",\"observability_ratio\":" << result.observability_ratio
             << ",\"alternate_overlap_ratio\":" << result.alternate_overlap_ratio
             << ",\"alternate_yaw_separation_rad\":"
             << result.alternate_yaw_separation_rad
             << ",\"odometry_prior_yaw_error_rad\":"
             << result.odometry_prior_yaw_error_rad
             << ",\"odometry_prior_translation_error_m\":"
             << result.odometry_prior_translation_error_m
             << ",\"rejection_reason\":\"" << result.rejection_reason << "\"}\n";
    }
    std::cout << "{\"schema_version\":1,\"pairs\":" << pairs.size()
              << ",\"available\":" << available
              << ",\"accepted\":" << accepted
              << ",\"matching_mode\":\""
              << (submap_builder ? "scan_to_submap" : "scan_to_scan") << "\""
              << ",\"effective_point_stride\":"
              << options.matcher.point_stride *
        (submap_builder ? options.submap.point_stride : 1U)
              << ",\"matcher_minimum_points\":" << options.matcher.minimum_points
              << ",\"submaps_built\":" << submap_cache.size() << "}\n";
    return available > 0U ? 0 : 1;
  } catch (const std::exception & exception) {
    std::cerr << "lidar_shadow_scan_match: " << exception.what() << '\n';
    return 2;
  }
}
