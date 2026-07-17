"""Voice 运行时探针与 pytest 断言的所有权契约。"""

from __future__ import annotations

import ast
from pathlib import Path

from repository_test_support import ROOT


TEST_ROOT = ROOT / "tests" / "integration" / "voice"
PROBE_ROOT = ROOT / "tools" / "acceptance" / "probes" / "voice"

EXECUTABLE_PROBES = {
    "continuous_command_ttl.py",
    "continuous_endpoint_asr.py",
    "continuous_kws_sidecar.py",
    "continuous_multi_command.py",
    "continuous_queue_full.py",
    "continuous_session_timeout.py",
    "continuous_voice_control.py",
    "continuous_voice_soak.py",
    "gazebo_voice.py",
    "keyword_wake_sidecar.py",
    "livekit_wakeword_sidecar.py",
    "offline_sherpa_typed_simulation.py",
    "offline_voice_e2e.py",
    "online_api.py",
    "openwakeword_sidecar.py",
    "recognition_retry.py",
    "sherpa_speaker_identity_ros.py",
    "speaker_enrollment.py",
    "speaker_memory_mock.py",
}


def _imports(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)
    return imports


def test_voice_tests_are_pytest_modules_not_hidden_runtime_programs():
    """tests 只保存 pytest 断言，避免“仅导入 main()”制造假绿色。"""

    test_modules = sorted(
        path for path in TEST_ROOT.glob("*.py") if path.name != "__init__.py"
    )
    assert test_modules
    for path in test_modules:
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
        assert path.name.startswith("test_")
        assert "__main__" not in source
        assert any(
            isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name.startswith("test_")
            for node in tree.body
        ), f"{path.name} 必须包含可被 pytest 收集的测试函数"


def test_voice_runtime_programs_are_owned_by_acceptance_tools():
    """真实 ROS/provider 探针由 smoke 显式执行，不参与 pytest 自动收集。"""

    probe_names = {
        path.name for path in PROBE_ROOT.glob("*.py") if path.name != "__init__.py"
    }
    # 集合相等让新增 probe 必须同时进入结构契约，不能绕过命名、入口和规模约束。
    assert EXECUTABLE_PROBES == probe_names

    for name in probe_names:
        probe = PROBE_ROOT / name
        source = probe.read_text(encoding="utf-8")
        assert not name.startswith("test_")
        assert 'if __name__ == "__main__"' in source
        assert not (TEST_ROOT / f"test_{name}").exists()


def test_voice_probes_do_not_depend_on_test_implementation_details():
    """运行工具只能依赖生产 Module/公共 Adapter，不能反向导入 tests。"""

    for name in EXECUTABLE_PROBES:
        assert all(
            not imported.startswith("tests.")
            for imported in _imports(PROBE_ROOT / name)
        )


def test_voice_probes_remain_focused_adapters():
    """Probe 只负责编排和观测；超过上限时应抽取可独立测试的深 Module。"""

    for name in EXECUTABLE_PROBES:
        line_count = len((PROBE_ROOT / name).read_text(encoding="utf-8").splitlines())
        assert line_count <= 350, f"{name} grew to {line_count} lines"


def test_speaker_memory_probe_reuses_production_topic_and_qos_contracts():
    """身份状态必须 transient-local，TTS topic 也不能由 probe 自行猜测。"""

    source = (PROBE_ROOT / "speaker_memory_mock.py").read_text(encoding="utf-8")

    assert "AgentTopicContract" in source
    assert "TOPICS.speaker_identity" in source
    assert "TOPICS.tts_pcm" in source
    assert "state_qos()" in source
    assert '"/agent/tts_pcm"' not in source


def test_real_audio_probes_use_the_current_endpoint_contract():
    """主端点启用时 silence_timeout 会被忽略，probe 必须发布 speech_ended。"""

    for name in (
        "gazebo_voice.py",
        "offline_sherpa_typed_simulation.py",
        "offline_voice_e2e.py",
    ):
        source = (PROBE_ROOT / name).read_text(encoding="utf-8")
        assert "AgentTopicContract" in source
        assert "TOPICS.speech_ended" in source
        assert '"/audio/silence_timeout"' not in source


def test_gazebo_voice_smoke_is_typed_action_only():
    """项目已删除 legacy JSON 动作链，验收入口不得再回退到无 scheduler 模式。"""

    source = (ROOT / "scripts/smoke_test_gazebo_voice.sh").read_text(
        encoding="utf-8"
    )

    assert "use_typed_actions:=true" in source
    assert "REQUIRE_TYPED_ACTION_RESULT" not in source
    assert "USE_TYPED_ACTIONS" not in source
