#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "embodied_slam/lidar_loop_temporal_consistency.hpp"

namespace
{

constexpr double kSecondsToNanoseconds = 1.0e9;

struct ReplayOptions
{
  embodied_slam::LidarLoopTemporalConsistencyConfig temporal;
};

void usage(const char * program)
{
  std::cerr << "Usage: " << program << " INPUT OUTPUT "
            << "[--minimum-confirmations N --maximum-query-gap S "
            << "--maximum-pair-age-delta S --maximum-translation-delta M "
            << "--maximum-yaw-delta RAD]\n";
}

ReplayOptions parseOptions(const int argc, char ** argv)
{
  ReplayOptions options;
  for (int index = 3; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--minimum-confirmations") {
      options.temporal.minimum_confirmations = std::stoul(value);
    } else if (argument == "--maximum-query-gap") {
      options.temporal.maximum_query_gap_s = std::stod(value);
    } else if (argument == "--maximum-pair-age-delta") {
      options.temporal.maximum_pair_age_delta_s = std::stod(value);
    } else if (argument == "--maximum-translation-delta") {
      options.temporal.maximum_translation_delta_m = std::stod(value);
    } else if (argument == "--maximum-yaw-delta") {
      options.temporal.maximum_yaw_delta_rad = std::stod(value);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  return options;
}

void writeBoolean(std::ostream & output, const bool value)
{
  output << (value ? "true" : "false");
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3) {
    usage(argv[0]);
    return 2;
  }
  try {
    const auto options = parseOptions(argc, argv);
    std::ifstream input(argv[1]);
    std::ofstream output(argv[2], std::ios::out | std::ios::trunc);
    if (!input.is_open() || !output.is_open()) {
      throw std::runtime_error("cannot open temporal replay input/output");
    }
    embodied_slam::LidarLoopTemporalConsistency consistency(options.temporal);
    output << std::setprecision(12);
    std::size_t observations = 0U;
    std::size_t approved = 0U;
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty() || line.front() == '#') {
        continue;
      }
      std::istringstream stream(line);
      char kind = '\0';
      embodied_slam::LidarLoopTemporalObservation observation;
      double query_stamp_s = 0.0;
      double candidate_stamp_s = 0.0;
      if (!(stream >> kind >> observation.query_id >> observation.candidate_id >>
        query_stamp_s >> candidate_stamp_s >> observation.target_to_source.x >>
        observation.target_to_source.y >> observation.target_to_source.yaw) || kind != 'O')
      {
        throw std::runtime_error("invalid temporal observation at line " +
                std::to_string(observations + 1U));
      }
      std::string trailing;
      if (stream >> trailing) {
        throw std::runtime_error("unexpected temporal observation columns");
      }
      observation.query_stamp_ns = static_cast<std::int64_t>(
        query_stamp_s * kSecondsToNanoseconds);
      observation.candidate_stamp_ns = static_cast<std::int64_t>(
        candidate_stamp_s * kSecondsToNanoseconds);
      const auto decision = consistency.observe(observation);
      ++observations;
      approved += decision.approved ? 1U : 0U;
      output << "{\"schema_version\":1,\"query_id\":" << observation.query_id
             << ",\"candidate_id\":" << observation.candidate_id
             << ",\"approved\":";
      writeBoolean(output, decision.approved);
      output << ",\"confirmation_count\":" << decision.confirmation_count
             << ",\"confirmation_required\":" << decision.confirmation_required
             << ",\"query_gap_s\":" << decision.query_gap_s
             << ",\"pair_age_delta_s\":" << decision.pair_age_delta_s
             << ",\"translation_delta_m\":" << decision.translation_delta_m
             << ",\"yaw_delta_rad\":" << decision.yaw_delta_rad
             << ",\"reason\":\"" << decision.reason << "\"}\n";
    }
    if (observations == 0U) {
      throw std::runtime_error("temporal replay input is empty");
    }
    std::cout << "{\"schema_version\":1,\"observations\":" << observations
              << ",\"approved\":" << approved << "}\n";
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << "lidar_loop_temporal_replay: " << exception.what() << '\n';
    return 2;
  }
}
