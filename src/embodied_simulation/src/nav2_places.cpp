#include "embodied_simulation/nav2_places.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace embodied_simulation
{
namespace
{

std::string trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n\"'");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n\"'");
  return value.substr(first, last - first + 1);
}

std::optional<double> field_value(
  const std::string & body, const std::string & field)
{
  const std::regex pattern(
    field + R"(\s*:\s*(-?(?:\d+(?:\.\d*)?|\.\d+)))");
  std::smatch match;
  if (!std::regex_search(body, match, pattern)) {
    return std::nullopt;
  }
  return std::stod(match[1].str());
}

}  // namespace

void Nav2Places::set_frame_id(std::string frame_id)
{
  frame_id_ = trim(std::move(frame_id));
  if (frame_id_.empty()) {
    frame_id_ = "map";
  }
}

const std::string & Nav2Places::frame_id() const
{
  return frame_id_;
}

void Nav2Places::add_place(const std::string & name, const Nav2Place & place)
{
  places_[trim(name)] = place;
}

std::optional<Nav2Place> Nav2Places::resolve(const std::string & name) const
{
  const auto iter = places_.find(trim(name));
  if (iter == places_.end()) {
    return std::nullopt;
  }
  return iter->second;
}

std::vector<std::string> Nav2Places::names() const
{
  std::vector<std::string> result;
  result.reserve(places_.size());
  for (const auto & item : places_) {
    result.push_back(item.first);
  }
  std::sort(result.begin(), result.end());
  return result;
}

geometry_msgs::msg::PoseStamped Nav2Places::to_pose_stamped(
  const std::string & name) const
{
  const auto place = resolve(name);
  if (!place) {
    throw std::out_of_range("unknown Nav2 place: " + name);
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = place->x;
  pose.pose.position.y = place->y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.z = std::sin(place->yaw / 2.0);
  pose.pose.orientation.w = std::cos(place->yaw / 2.0);
  return pose;
}

Nav2Places load_nav2_places_yaml(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open Nav2 places file: " + path);
  }
  Nav2Places places;
  std::string line;
  const std::regex frame_pattern(R"(^\s*frame_id\s*:\s*(.+?)\s*$)");
  const std::regex place_pattern(
    R"(^\s*([A-Za-z0-9_]+)\s*:\s*\{(.+)\}\s*$)");
  while (std::getline(stream, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    std::smatch match;
    if (std::regex_match(line, match, frame_pattern)) {
      places.set_frame_id(match[1].str());
      continue;
    }
    if (!std::regex_match(line, match, place_pattern)) {
      continue;
    }
    const std::string name = match[1].str();
    const std::string body = match[2].str();
    const auto x = field_value(body, "x");
    const auto y = field_value(body, "y");
    const auto yaw = field_value(body, "yaw");
    if (!x || !y || !yaw) {
      throw std::runtime_error("invalid Nav2 place entry: " + name);
    }
    places.add_place(name, Nav2Place{*x, *y, *yaw});
  }
  return places;
}

}  // namespace embodied_simulation
