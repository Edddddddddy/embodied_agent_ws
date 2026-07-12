#include <gtest/gtest.h>

#include "embodied_agent_middleware/component_health_registry.hpp"

namespace
{

using embodied_agent_middleware::ComponentHealthRegistry;
using embodied_agent_middleware::ComponentState;
using embodied_agent_middleware::parse_component_csv;

TEST(ComponentHealthRegistryTest, RequiresEveryFreshReadyComponent)
{
  ComponentHealthRegistry registry;
  registry.update({"agent", ComponentState::kReady, "online", 10.0});
  registry.update({"action_guard", ComponentState::kReady, "matched", 10.0});

  const auto ready = registry.evaluate({"agent", "action_guard"}, 11.0, 3.0);
  EXPECT_TRUE(ready.ready);
  EXPECT_EQ(ready.detail, "all_required_components_ready");

  const auto stale = registry.evaluate({"agent", "action_guard"}, 14.0, 3.0);
  EXPECT_FALSE(stale.ready);
  EXPECT_EQ(stale.missing_components.size(), 2U);
}

TEST(ComponentHealthRegistryTest, KeepsDegradedDistinctFromMissing)
{
  ComponentHealthRegistry registry;
  registry.update({"agent", ComponentState::kDegraded, "provider_retry", 1.0});

  const auto output = registry.evaluate({"agent", "simulation"}, 1.2, 3.0);
  EXPECT_EQ(output.degraded_components, std::vector<std::string>({"agent"}));
  EXPECT_EQ(output.missing_components, std::vector<std::string>({"simulation"}));
  EXPECT_EQ(output.detail, "required_component_degraded");
}

TEST(ComponentHealthRegistryTest, CsvParserTrimsDeduplicatesAndDropsEmptyItems)
{
  EXPECT_EQ(
    parse_component_csv(" agent, action_guard,agent, ,simulation "),
    std::vector<std::string>({"agent", "action_guard", "simulation"}));
}

}  // namespace
