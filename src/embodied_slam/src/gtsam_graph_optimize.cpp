#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "embodied_slam/gtsam_pose_graph.hpp"

namespace
{

struct GraphSnapshot
{
  std::unordered_map<int, embodied_slam::Pose2d> poses;
  std::unordered_map<int, double> stamps;
  std::vector<embodied_slam::PoseGraphConstraint> constraints;
};

GraphSnapshot load_graph(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open graph snapshot: " + path);
  }
  GraphSnapshot graph;
  std::string raw;
  std::size_t line_number = 0U;
  while (std::getline(input, raw)) {
    ++line_number;
    if (raw.empty() || raw.front() == '#') {
      continue;
    }
    std::istringstream row(raw);
    char kind = '\0';
    row >> kind;
    if (kind == 'N') {
      int id = 0;
      double stamp = 0.0;
      embodied_slam::Pose2d pose;
      if (!(row >> id >> stamp >> pose.x >> pose.y >> pose.yaw)) {
        throw std::runtime_error("invalid node row at line " + std::to_string(line_number));
      }
      graph.poses[id] = pose;
      graph.stamps[id] = stamp;
    } else if (kind == 'C') {
      embodied_slam::PoseGraphConstraint constraint;
      if (!(row >> constraint.source_id >> constraint.target_id >> constraint.relative_pose.x >>
        constraint.relative_pose.y >> constraint.relative_pose.yaw))
      {
        throw std::runtime_error(
                "invalid constraint row at line " + std::to_string(line_number));
      }
      for (std::size_t row_index = 0; row_index < 3U; ++row_index) {
        for (std::size_t column = 0; column < 3U; ++column) {
          if (!(row >> constraint.covariance(row_index, column))) {
            throw std::runtime_error(
                    "incomplete covariance at line " + std::to_string(line_number));
          }
        }
      }
      graph.constraints.push_back(constraint);
    } else {
      throw std::runtime_error("unknown graph row at line " + std::to_string(line_number));
    }
    row >> std::ws;
    if (!row.eof()) {
      throw std::runtime_error("extra fields at line " + std::to_string(line_number));
    }
  }
  if (graph.poses.size() < 3U || graph.constraints.empty()) {
    throw std::runtime_error("graph snapshot must contain at least 3 nodes and 1 constraint");
  }
  for (const auto & constraint : graph.constraints) {
    if (graph.poses.count(constraint.source_id) == 0U ||
      graph.poses.count(constraint.target_id) == 0U)
    {
      throw std::runtime_error("constraint references a missing node");
    }
  }
  return graph;
}

void write_tum(
  const std::string & path, const embodied_slam::PoseGraphResult & result,
  const std::unordered_map<int, double> & stamps)
{
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot open trajectory output: " + path);
  }
  std::vector<int> ids;
  ids.reserve(result.poses.size());
  for (const auto & [id, pose] : result.poses) {
    (void)pose;
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  output << "# timestamp tx ty tz qx qy qz qw\n" << std::setprecision(17);
  for (const int id : ids) {
    const auto & pose = result.poses.at(id);
    const double half_yaw = pose.yaw * 0.5;
    output << stamps.at(id) << ' ' << pose.x << ' ' << pose.y
           << " 0 0 0 " << std::sin(half_yaw) << ' ' << std::cos(half_yaw) << '\n';
  }
  if (!output) {
    throw std::runtime_error("failed while writing trajectory: " + path);
  }
}

bool parse_bool(const std::string & value)
{
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::invalid_argument("expected true/false, got: " + value);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 8 && argc != 11) {
    std::cerr << "Usage: gtsam_graph_optimize GRAPH OUTPUT_TUM KERNEL K "
              << "LOOP_ONLY LOOP_ID_SEPARATION MAX_ITERATIONS "
              << "[CONSISTENCY_GATE MAX_TRANSLATION_RESIDUAL_M MAX_YAW_RESIDUAL_RAD]\n";
    return 2;
  }
  try {
    const GraphSnapshot graph = load_graph(argv[1]);
    embodied_slam::PoseGraphOptimizerConfig config;
    config.robust_kernel = embodied_slam::robust_kernel_from_string(argv[3]);
    config.robust_kernel_k = std::stod(argv[4]);
    config.robustify_loop_constraints_only = parse_bool(argv[5]);
    config.loop_constraint_min_id_separation = std::stoul(argv[6]);
    config.max_iterations = std::stoul(argv[7]);
    if (argc == 11) {
      config.enable_nonlocal_consistency_gate = parse_bool(argv[8]);
      config.max_nonlocal_translation_residual_m = std::stod(argv[9]);
      config.max_nonlocal_yaw_residual_rad = std::stod(argv[10]);
    }
    const embodied_slam::PoseGraphResult result =
      embodied_slam::GtsamPoseGraphOptimizer(config).optimize(graph.poses, graph.constraints);
    write_tum(argv[2], result, graph.stamps);
    // 标准输出仅包含机器可读摘要，便于 shell/Python 消融器保存每个 variant 的审计数据。
    std::cout << std::setprecision(17)
              << "{\"schema_version\":1,\"kernel\":\""
              << embodied_slam::robust_kernel_name(config.robust_kernel)
              << "\",\"kernel_k\":" << config.robust_kernel_k
              << ",\"loop_only\":"
              << (config.robustify_loop_constraints_only ? "true" : "false")
              << ",\"nodes\":" << result.poses.size()
              // constraints 表示不可变输入图规模，用于消融公平性检查；实际采用数量单独报告。
              << ",\"constraints\":" << graph.constraints.size()
              << ",\"constraints_used\":" << result.constraints_used
              << ",\"robustified_constraints\":" << result.robustified_constraints
              << ",\"consistency_gate\":"
              << (config.enable_nonlocal_consistency_gate ? "true" : "false")
              << ",\"consistency_rejected_constraints\":"
              << result.consistency_rejected_constraints
              << ",\"initial_error\":" << result.initial_error
              << ",\"final_error\":" << result.final_error
              << ",\"iterations\":" << result.iterations << "}\n";
  } catch (const std::exception & error) {
    std::cerr << "gtsam_graph_optimize: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
