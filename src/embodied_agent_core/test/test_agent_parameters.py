from types import SimpleNamespace

import pytest

from embodied_agent_core.agent_parameters import (
    AgentParameterError,
    COMMON_PARAMETER_SPECS,
    declare_agent_parameters,
    default_parameter_values,
    validate_parameter_values,
)


class _FakeNode:
    def __init__(self, overrides=None):
        self._overrides = overrides or {}
        self.values = {}
        self.descriptors = {}

    def declare_parameter(self, name, default, descriptor):
        self.values[name] = self._overrides.get(name, default)
        self.descriptors[name] = descriptor

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


def test_common_defaults_do_not_drift_between_online_and_offline_profiles():
    online = default_parameter_values("online")
    offline = default_parameter_values("offline")

    profile_specific = {
        "memory_path",
        "memory_max_turns",
        "llm_context_window_tokens",
        "prompt_safety_reserve_tokens",
        "rag_max_context_chars",
        "rag_cloud_context_policy",
    }
    for spec in COMMON_PARAMETER_SPECS:
        if spec.name not in profile_specific:
            assert online[spec.name] == offline[spec.name]


def test_profile_specific_defaults_remain_explicit():
    online = default_parameter_values("online")
    offline = default_parameter_values("offline")

    assert online["mode"] == "mock"
    assert offline["mode"] == "mock"
    assert online["memory_max_turns"] == 10
    assert offline["memory_max_turns"] == 3
    assert online["tts_sample_rate"] == 24000
    assert offline["tts_sample_rate"] == 44100
    assert offline["asr_tail_padding_s"] == 0.66
    assert offline["asr_max_active_paths"] == 16
    assert online["rag_max_context_chars"] == 1800
    assert offline["rag_max_context_chars"] == 1200
    assert offline["rag_query_policy"] == "adaptive"
    assert online["rag_cloud_context_policy"] == "builtin_only"
    assert offline["rag_cloud_context_policy"] == "allow_custom"
    assert online["llm_context_window_tokens"] == 32768
    assert offline["llm_context_window_tokens"] == 2048
    assert online["llm_max_tokens"] == 512
    assert offline["llm_max_tokens"] == 192
    assert online["prompt_safety_reserve_tokens"] == 512
    assert offline["prompt_safety_reserve_tokens"] == 256


@pytest.mark.parametrize(
    ("profile", "override", "field"),
    [
        ("online", {"continuous_command_queue_size": 0}, "continuous_command_queue_size"),
        ("offline", {"tts_provider": "unknown"}, "tts_provider"),
        (
            "online",
            {"command_normalization_fuzzy_threshold": 1.1},
            "command_normalization_fuzzy_threshold",
        ),
        ("offline", {"asr_num_threads": 0}, "asr_num_threads"),
        ("offline", {"asr_tail_padding_s": -0.01}, "asr_tail_padding_s"),
        ("offline", {"asr_tail_padding_s": 2.01}, "asr_tail_padding_s"),
        ("online", {"rag_query_policy": "invalid"}, "rag_query_policy"),
        (
            "online",
            {"rag_cloud_context_policy": "invalid"},
            "rag_cloud_context_policy",
        ),
        (
            "offline",
            {"rag_chunk_chars": 600, "rag_chunk_overlap_chars": 600},
            "rag_chunk_overlap_chars",
        ),
        (
            "offline",
            {
                "llm_context_window_tokens": 448,
                "llm_max_tokens": 192,
                "prompt_safety_reserve_tokens": 256,
            },
            "llm_context_window_tokens",
        ),
    ],
)
def test_invalid_parameter_fails_with_field_name(profile, override, field):
    values = default_parameter_values(profile)
    values.update(override)

    with pytest.raises(AgentParameterError, match=field):
        validate_parameter_values(profile, values)


def test_partial_age_must_cover_commit_delay():
    values = default_parameter_values("online")
    values.update(asr_commit_delay_ms=2500, asr_partial_max_age_s=2.0)

    with pytest.raises(AgentParameterError, match="asr_partial_max_age_s"):
        validate_parameter_values("online", values)


def test_declare_returns_validated_immutable_snapshot_and_ros_descriptors():
    node = _FakeNode({"continuous_command_queue_size": 12})
    parameters = declare_agent_parameters(node, "online")

    assert parameters.get("continuous_command_queue_size") == 12
    assert "命令队列容量" in node.descriptors["continuous_command_queue_size"].description
    assert node.descriptors["continuous_command_queue_size"].read_only is True
    assert node.descriptors["continuous_command_queue_size"].integer_range[0].from_value == 1
    with pytest.raises(TypeError):
        parameters.values["mode"] = "online"


def test_unknown_profile_is_rejected():
    with pytest.raises(AgentParameterError, match="未知"):
        default_parameter_values("edge")
