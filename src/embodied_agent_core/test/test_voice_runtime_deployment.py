from pathlib import Path

import pytest

from embodied_agent_core.voice_runtime_deployment import (
    VoiceRuntimeDeploymentChecker,
    VoiceRuntimeProfileError,
)


def _profile(tmp_path: Path):
    knowledge = tmp_path / "knowledge.md"
    knowledge.write_text("robot deployment", encoding="utf-8")
    return {
        "components": {
            "vad": {
                "required": True,
                "candidates": [
                    {
                        "provider": "silero",
                        "python_modules": ["onnxruntime"],
                        "paths": [
                            {
                                "kind": "file",
                                "value": "${RUNTIME_ROOT}/silero.onnx",
                            }
                        ],
                    },
                    {"provider": "energy"},
                ],
            },
            "rag": {
                "required": True,
                "candidates": [
                    {
                        "provider": "sparse",
                        "paths": [
                            {
                                "kind": "file",
                                "value": "${WORKSPACE}/knowledge.md",
                            }
                        ],
                    }
                ],
            },
            "llm": {
                "required": True,
                "candidates": [
                    {
                        "provider": "llama_cpp",
                        "endpoints": ["http://127.0.0.1:8080/health"],
                    }
                ],
            },
        }
    }


def test_checker_selects_fallback_and_does_not_probe_network_by_default(tmp_path):
    endpoint_calls = []
    checker = VoiceRuntimeDeploymentChecker(
        module_finder=lambda _name: None,
        endpoint_probe=lambda url, timeout: endpoint_calls.append((url, timeout)),
    )

    report = checker.check(
        "offline-edge",
        _profile(tmp_path),
        variables={"WORKSPACE": str(tmp_path), "RUNTIME_ROOT": str(tmp_path)},
    )

    assert report.ok
    assert report.status == "degraded"
    assert report.components[0].selected_provider == "energy"
    assert "fallback_selected:energy" in report.components[0].warnings[0]
    assert "endpoint_not_probed" in report.components[2].warnings[0]
    assert endpoint_calls == []


def test_unprobed_endpoint_is_reported_as_unverified_not_ready(tmp_path):
    checker = VoiceRuntimeDeploymentChecker()
    profile = {
        "components": {
            "llm": {
                "required": True,
                "candidates": [
                    {
                        "provider": "llama_cpp",
                        "endpoints": ["http://127.0.0.1:8080/health"],
                    }
                ],
            }
        }
    }

    report = checker.check("offline-edge", profile, variables={})

    assert report.ok
    assert report.status == "degraded"
    assert report.components[0].status == "unverified"
    assert "endpoint_not_probed" in report.components[0].warnings[0]


def test_cloud_credentials_are_never_treated_as_runtime_health():
    checker = VoiceRuntimeDeploymentChecker(module_finder=lambda _name: object())
    profile = {
        "components": {
            "asr": {
                "required": True,
                "candidates": [
                    {
                        "provider": "cloud_asr",
                        "python_modules": ["dashscope"],
                        "environment": ["DASHSCOPE_API_KEY"],
                        "verification": "credential_only",
                    }
                ],
            }
        }
    }

    report = checker.check(
        "online-cloud",
        profile,
        variables={},
        environment={"DASHSCOPE_API_KEY": "test-placeholder"},
    )

    assert report.ok
    assert report.status == "degraded"
    assert report.components[0].status == "unverified"
    assert "cloud_credentials_not_verified" in report.components[0].warnings[0]


def test_required_asset_and_environment_failures_are_component_blockers(tmp_path):
    profile = {
        "components": {
            "asr": {
                "required": True,
                "candidates": [
                    {
                        "provider": "cloud_asr",
                        "python_modules": ["dashscope"],
                        "environment": ["DASHSCOPE_API_KEY"],
                        "paths": [
                            {"kind": "file", "value": "${WORKSPACE}/missing.bin"}
                        ],
                    }
                ],
            }
        }
    }
    checker = VoiceRuntimeDeploymentChecker(module_finder=lambda _name: None)

    report = checker.check(
        "online-cloud",
        profile,
        variables={"WORKSPACE": str(tmp_path)},
        environment={},
    )

    assert not report.ok
    assert report.status == "blocked"
    assert any("python_module_missing:dashscope" in item for item in report.blockers)
    assert any(
        "environment_missing:DASHSCOPE_API_KEY" in item
        for item in report.blockers
    )
    assert any("path_file_missing" in item for item in report.blockers)


def test_contract_only_validates_schema_without_touching_host_dependencies(tmp_path):
    checker = VoiceRuntimeDeploymentChecker(
        module_finder=lambda _name: (_ for _ in ()).throw(
            AssertionError("module finder must not run")
        ),
        path_exists=lambda _path: (_ for _ in ()).throw(
            AssertionError("path check must not run")
        ),
        endpoint_probe=lambda _url, _timeout: (_ for _ in ()).throw(
            AssertionError("endpoint probe must not run")
        ),
    )

    report = checker.check(
        "offline-edge",
        _profile(tmp_path),
        variables={},
        contract_only=True,
    )

    assert report.ok
    assert report.status == "contract_valid"
    assert all(component.status == "contract" for component in report.components)


def test_explicit_endpoint_probe_turns_unhealthy_runtime_into_blocker(tmp_path):
    checker = VoiceRuntimeDeploymentChecker(
        module_finder=lambda _name: None,
        endpoint_probe=lambda _url, _timeout: False,
    )

    report = checker.check(
        "offline-edge",
        _profile(tmp_path),
        variables={"WORKSPACE": str(tmp_path), "RUNTIME_ROOT": str(tmp_path)},
        probe_endpoints=True,
    )

    assert not report.ok
    assert any("endpoint_unhealthy" in item for item in report.blockers)


def test_invalid_component_contract_is_rejected():
    checker = VoiceRuntimeDeploymentChecker()

    with pytest.raises(VoiceRuntimeProfileError, match="candidate"):
        checker.check(
            "bad",
            {"components": {"asr": {"required": True, "candidates": ["bad"]}}},
            variables={},
            contract_only=True,
        )

    with pytest.raises(VoiceRuntimeProfileError, match="unknown path kind"):
        checker.check(
            "bad",
            {
                "components": {
                    "asr": {
                        "candidates": [
                            {
                                "provider": "bad",
                                "paths": [{"kind": "socket", "value": "/tmp/asr"}],
                            }
                        ]
                    }
                }
            },
            variables={},
            contract_only=True,
        )

    with pytest.raises(VoiceRuntimeProfileError, match="unknown verification"):
        checker.check(
            "bad",
            {
                "components": {
                    "asr": {
                        "candidates": [
                            {
                                "provider": "bad",
                                "verification": "trust_me",
                            }
                        ]
                    }
                }
            },
            variables={},
            contract_only=True,
        )
