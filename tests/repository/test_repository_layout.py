"""仓库结构约束：用户命令与集成测试必须分区，避免 scripts/ 再次退化成杂物箱。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_integration_probes_are_not_mixed_with_user_scripts():
    misplaced = sorted((ROOT / "scripts").glob("test_*"))
    assert misplaced == [], f"测试探针应放入 tests/integration: {misplaced}"


def test_critical_full_chain_probes_remain_discoverable():
    integration = ROOT / "tests" / "integration"
    required = {
        "test_gazebo_voice.py",
        "test_mock_executor_pipeline.py",
        "test_online_api.py",
        "test_recognition_retry.py",
        "test_continuous_command_ttl.py",
        "test_typed_action_server.py",
    }
    present = {path.name for path in integration.glob("test_*")}
    assert required <= present
