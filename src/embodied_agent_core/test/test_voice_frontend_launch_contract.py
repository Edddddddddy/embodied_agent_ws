from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from embodied_agent_core.voice_frontend_launch_contract import (
    VOICE_FRONTEND_ARGUMENTS,
    declare_voice_frontend_arguments,
    voice_frontend_configurations,
    voice_frontend_nodes,
)


def test_voice_frontend_arguments_have_one_unique_typed_contract():
    names = [spec.name for spec in VOICE_FRONTEND_ARGUMENTS]
    assert len(names) == len(set(names))
    assert set(voice_frontend_configurations()) == set(names)
    assert {"capture_enabled", "vad_provider", "kws_provider"} <= set(names)


def test_capture_default_can_follow_agent_microphone_launch_configuration():
    microphone = LaunchConfiguration("microphone_enabled")
    arguments = {
        action.name: action for action in declare_voice_frontend_arguments(microphone)
    }
    assert arguments["capture_enabled"].default_value == [microphone]


def test_shared_contract_builds_exactly_five_frontend_nodes():
    nodes = voice_frontend_nodes(LaunchConfiguration("config"))
    assert len(nodes) == 5
    assert all(isinstance(node, Node) for node in nodes)
