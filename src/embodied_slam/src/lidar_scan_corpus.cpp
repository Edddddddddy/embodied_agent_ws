#include "embodied_slam/lidar_scan_corpus.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace embodied_slam
{

std::vector<LidarScanRecord> loadLidarScanCorpus(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("cannot open scan corpus: " + path);
  }
  std::vector<LidarScanRecord> rows;
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
    LidarScanRecord row;
    if (!(stream >> kind >> row.scan_id >> row.stamp_s >> point_count) || kind != 'S') {
      throw std::runtime_error("invalid corpus row at line " + std::to_string(line_number));
    }
    row.points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      LidarPoint2D point;
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

}  // namespace embodied_slam
