#include <cmath>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "embodied_simulation/nav2_places.hpp"

namespace embodied_simulation
{
namespace
{

TEST(Nav2PlacesTest, LoadsSemanticPlacesFromYaml)
{
  const std::string path = "/tmp/embodied_nav2_places_test.yaml";
  {
    std::ofstream stream(path);
    stream << "frame_id: map\n";
    stream << "places:\n";
    stream << "  home: {x: 0.0, y: 0.0, yaw: 0.0}\n";
    stream << "  door: {x: 1.2, y: -0.5, yaw: 1.57}\n";
  }

  const auto places = load_nav2_places_yaml(path);
  EXPECT_EQ(places.frame_id(), "map");
  ASSERT_TRUE(places.resolve("door").has_value());
  EXPECT_NEAR(places.resolve("door")->x, 1.2, 1e-9);
  EXPECT_NEAR(places.resolve("door")->y, -0.5, 1e-9);

  const auto pose = places.to_pose_stamped("door");
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_NEAR(pose.pose.position.x, 1.2, 1e-9);
  EXPECT_NEAR(pose.pose.position.y, -0.5, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.z, std::sin(1.57 / 2.0), 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, std::cos(1.57 / 2.0), 1e-9);
}

TEST(Nav2PlacesTest, RejectsUnknownPlace)
{
  Nav2Places places;
  places.add_place("home", Nav2Place{0.0, 0.0, 0.0});
  EXPECT_FALSE(places.resolve("door").has_value());
  EXPECT_THROW(places.to_pose_stamped("door"), std::out_of_range);
}

}  // namespace
}  // namespace embodied_simulation
