from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from embodied_agent_bringup.agent_launch_contract import (
    AGENT_CONTROL_ARGUMENTS,
    FORWARDED_AGENT_ARGUMENT_NAMES,
    agent_control_configurations,
    agent_control_parameter_overrides,
    declare_agent_control_arguments,
    declare_forwarded_agent_arguments,
    forwarded_agent_launch_arguments,
)


def test_control_argument_names_are_unique_and_shared_by_both_profiles():
    names = [spec.name for spec in AGENT_CONTROL_ARGUMENTS]
    assert len(names) == len(set(names))

    online = declare_agent_control_arguments("online")
    offline = declare_agent_control_arguments("offline")
    assert [action.name for action in online] == names
    assert [action.name for action in offline] == names


def test_profile_specific_memory_default_is_preserved_in_launch_contract():
    online = {action.name: action for action in declare_agent_control_arguments("online")}
    offline = {action.name: action for action in declare_agent_control_arguments("offline")}

    online_default = "".join(item.text for item in online["memory_path"].default_value)
    offline_default = "".join(item.text for item in offline["memory_path"].default_value)
    assert online_default.endswith("/memory.json")
    assert offline_default.endswith("/offline_memory.json")


def test_launch_configurations_cover_exactly_the_public_contract():
    expected = {spec.name for spec in AGENT_CONTROL_ARGUMENTS}
    configurations = agent_control_configurations()
    overrides = agent_control_parameter_overrides()

    assert set(configurations) == expected
    assert set(overrides) == expected
    assert all(isinstance(value, LaunchConfiguration) for value in configurations.values())


def test_non_string_parameter_overrides_have_explicit_ros_types():
    types = {spec.name: spec.value_type for spec in AGENT_CONTROL_ARGUMENTS}
    overrides = agent_control_parameter_overrides()

    for name, value in overrides.items():
        if types[name] is str:
            assert isinstance(value, LaunchConfiguration)
        else:
            assert isinstance(value, ParameterValue)


def test_simulation_forward_contract_excludes_persistent_paths():
    assert "memory_path" not in FORWARDED_AGENT_ARGUMENT_NAMES
    assert "user_memory_dir" not in FORWARDED_AGENT_ARGUMENT_NAMES
    assert "microphone_enabled" in FORWARDED_AGENT_ARGUMENT_NAMES
    assert "asr_commit_delay_ms" in FORWARDED_AGENT_ARGUMENT_NAMES


def test_simulation_profile_can_override_experience_defaults():
    actions = declare_forwarded_agent_arguments(
        default_overrides={"voice_session_timeout_s": 120.0, "asr_commit_delay_ms": 300}
    )
    by_name = {action.name: action for action in actions}
    timeout = "".join(item.text for item in by_name["voice_session_timeout_s"].default_value)
    delay = "".join(item.text for item in by_name["asr_commit_delay_ms"].default_value)
    assert timeout == "120.0"
    assert delay == "300"


def test_online_and_offline_includes_receive_identical_forwarded_contract():
    provider_mode = LaunchConfiguration("provider_mode")
    forwarded = forwarded_agent_launch_arguments(provider_mode)
    assert forwarded["mode"] is provider_mode
    assert set(forwarded) == {"mode", *FORWARDED_AGENT_ARGUMENT_NAMES}
