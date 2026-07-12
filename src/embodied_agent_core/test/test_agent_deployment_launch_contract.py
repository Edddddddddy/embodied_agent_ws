from embodied_agent_core.agent_deployment_launch_contract import (
    AGENT_DEPLOYMENT_ARGUMENTS,
    agent_deployment_nodes,
    declare_agent_deployment_arguments,
)


def test_deployment_arguments_are_unique_and_complete():
    names = [spec.name for spec in AGENT_DEPLOYMENT_ARGUMENTS]
    assert len(names) == len(set(names))
    assert {action.name for action in declare_agent_deployment_arguments()} == set(names)


def test_deployment_contract_builds_guard_manager_and_hardware_adapter():
    assert len(agent_deployment_nodes("online_agent")) == 3
    assert len(agent_deployment_nodes("offline_agent")) == 3
