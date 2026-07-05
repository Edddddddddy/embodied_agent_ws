#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace embodied_simulation
{

struct Nav2Place
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class Nav2Places
{
public:
  void set_frame_id(std::string frame_id);
  const std::string & frame_id() const;
  void add_place(const std::string & name, const Nav2Place & place);
  std::optional<Nav2Place> resolve(const std::string & name) const;
  std::vector<std::string> names() const;
  geometry_msgs::msg::PoseStamped to_pose_stamped(
    const std::string & name) const;

private:
  std::string frame_id_{"map"};
  std::unordered_map<std::string, Nav2Place> places_;
};

Nav2Places load_nav2_places_yaml(const std::string & path);

}  // namespace embodied_simulation
