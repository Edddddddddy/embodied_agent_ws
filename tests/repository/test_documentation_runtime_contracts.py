"""文档中的运行时数据流和演示边界必须与当前代码一致。"""

import re
from pathlib import Path

from repository_test_support import ROOT


def _read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def _local_markdown_targets(document: Path):
    """只校验仓库内链接；网页链接和页内锚点由渲染器负责。"""
    text = document.read_text(encoding="utf-8")
    for raw_target in re.findall(r"!?\[[^]]*\]\(([^)]+)\)", text):
        target = raw_target.strip().strip("<>")
        if target.startswith(("#", "http://", "https://", "mailto:")):
            continue
        yield target.split("#", 1)[0]


def test_audio_topic_and_asr_code_anchors_match_runtime_sources():
    architecture = _read("docs/ARCHITECTURE.md")
    presentation = _read("docs/PRESENTATION_15MIN.md")

    assert "/audio/clean_pcm" in architecture
    assert re.search(r"/audio/clean(?!_pcm)", architecture) is None
    assert (
        "src/embodied_agent_interfaces/action/ExecuteRobotCommand.action"
        in presentation
    )
    assert (
        "src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py"
        in presentation
    )
    assert "AsrEndpointRuntime" in presentation
    assert "_commit_asr_endpoint()" in presentation
    assert "streaming_asr.py" not in presentation
    assert "急停旁路" not in presentation
    assert "报告中的新地图" not in presentation
    assert "报告只在事后记录地图来源" in presentation


def test_slam_initialization_and_command_transaction_are_documented_separately():
    architecture = _read("docs/ARCHITECTURE.md")

    assert "节点初始化" in architecture
    assert "命令事务" in architecture
    assert architecture.index("MissionConfiguration.load()") < architecture.index(
        "_start_mapping()"
    )
    assert architecture.index("_on_asr_final()") < architecture.index(
        "AutomaticMissionExecutor.run()"
    )


def test_nav2_executor_and_nav2_stack_have_distinct_diagram_roles():
    readme = _read("README.md")
    architecture = _read("docs/ARCHITECTURE.md")

    for document in (readme, architecture):
        assert "Nav2RobotExecutor" in document
        assert "Nav2 Action servers" in document
        assert "Gazebo sensors" in document
    assert "NAV2 --> SLAM" not in readme


def test_learning_notes_do_not_reference_removed_online_forwarder():
    voice_notes = _read("docs/learning/VOICE_AGENT.md")
    online_node = _read(
        "src/embodied_online_agent/embodied_online_agent/online_agent_node.py"
    )

    assert "OnlineAgentNode._accept_transcript()" not in voice_notes
    assert "`_accept_transcript()`" not in voice_notes
    assert "self._application.accept_transcript(text)" in online_node
    assert "self._application.accept_transcript(message.data)" in online_node


def test_slam_and_acceptance_docs_preserve_runtime_ownership_boundaries():
    architecture = _read("docs/ARCHITECTURE.md")
    testing = _read("docs/TESTING.md")
    acceptance_readme = _read("tools/acceptance/README.md")

    assert "SLAM Toolbox\\ncanonical mapping" in architecture
    assert "Ceres / GTSAM\\nexperimental evidence" in architecture
    assert "executor 不直接接受候选层 `ARC`" in architecture
    for document in (architecture, testing, acceptance_readme):
        assert "AcceptanceSession" in document
    assert "acceptance_session.json" in testing


def test_live_voice_and_deterministic_dynamic_evidence_are_not_conflated():
    testing = _read("docs/TESTING.md")
    presentation = _read("docs/PRESENTATION_15MIN.md")

    for document in (testing, presentation):
        assert "crossing_cart" in document
        assert "slam-nav-e2e" in document
        assert "不自动注入" in document
    assert "进入 E2E 探针后" in testing
    assert "最后观测速度" in testing
    assert "只有成功报告才要求 `final_cmd_vel_zero=true`" in testing


def test_navigation_evidence_index_classifies_aggregate_gate_separately():
    evidence = _read("docs/evidence/navigation/README.md")

    assert "会话级 E2E 证据" in evidence
    assert "聚合门禁" in evidence
    assert "不能独立证明" in evidence


def test_history_document_cannot_be_mistaken_for_current_project_truth():
    history = _read("docs/history/CHANGELOG.md")

    assert "版本历史归档" in history
    assert "不是当前架构、完成度或路线图的权威事实源" in history
    assert "evidence/architecture_facts.md" in history
    assert (ROOT / "docs/evidence/architecture_facts.md").is_file()
    assert "TESTING.md" in history
    assert "GitHub Issues" in history
    assert "docs/README.md" not in history


def test_authoritative_document_local_links_resolve_to_real_files():
    documents = (
        ROOT / "README.md",
        ROOT / "docs/README.md",
        ROOT / "docs/ARCHITECTURE.md",
        ROOT / "docs/PRESENTATION_15MIN.md",
        ROOT / "docs/TESTING.md",
        ROOT / "docs/learning/VOICE_AGENT.md",
        ROOT / "docs/learning/ROS2_CPP_CONTROL.md",
        ROOT / "docs/learning/SLAM_NAV2.md",
        ROOT / "docs/development/WSL_POWERSHELL.md",
        ROOT / "docs/evidence/navigation/README.md",
        ROOT / "docs/history/CHANGELOG.md",
    )

    missing = []
    for document in documents:
        for target in _local_markdown_targets(document):
            if not (document.parent / target).resolve().exists():
                missing.append(f"{document.relative_to(ROOT)} -> {target}")
    assert missing == []
