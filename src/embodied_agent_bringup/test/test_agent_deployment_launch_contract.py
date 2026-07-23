from embodied_agent_bringup.agent_deployment_launch_contract import (
    AGENT_DEPLOYMENT_ARGUMENTS,
    agent_deployment_nodes,
    control_authority_manager_node,
    declare_agent_deployment_arguments,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node


def test_deployment_arguments_are_unique_and_complete():
    names = [spec.name for spec in AGENT_DEPLOYMENT_ARGUMENTS]
    assert len(names) == len(set(names))
    assert {action.name for action in declare_agent_deployment_arguments()} == set(names)


def test_deployment_contract_builds_guard_manager_and_hardware_adapter():
    assert len(agent_deployment_nodes("online_agent")) == 3
    assert len(agent_deployment_nodes("offline_agent")) == 3


def test_control_authority_manager_has_one_shared_launch_factory():
    node = control_authority_manager_node(
        condition=IfCondition("true"),
        state_heartbeat_ms=200,
    )

    assert isinstance(node, Node)
    parameters = node._Node__parameters  # launch_ros 暂无公开只读访问器
    normalized = {
        "".join(substitution.text for substitution in key): value
        for parameter in parameters
        for key, value in parameter.items()
    }
    assert normalized["bootstrap_quiescence_acknowledged"] is False
