from pathlib import Path

from embodied_agent_core.command_fallback import parse_fallback_action
from embodied_agent_core.command_normalizer import CommandNormalizer, NormalizationRules


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def normalize(text: str):
    return CommandNormalizer().normalize(text)


def test_normalizes_common_asr_typos_into_executable_commands():
    assert normalize("钱进一秒").text == "前进一秒"
    assert parse_fallback_action(normalize("钱进一秒").text).name == "move"

    assert normalize("相前走一秒").text == "向前走一秒"
    assert parse_fallback_action(normalize("相前走一秒").text).name == "move"

    turn = parse_fallback_action(normalize("作转九十度").text)
    assert turn.name == "turn"
    assert turn.arguments["angular_z"] > 0

    assert parse_fallback_action(normalize("让圈").text).name == "arc"
    assert parse_fallback_action(normalize("证方形").text).name == "move"


def test_normalizes_priority_stop_before_session_decision():
    result = normalize("亭下")

    assert result.text == "停下"
    assert result.changed
    assert parse_fallback_action(result.text).name == "stop"


def test_normalization_feedback_is_stable_json():
    result = normalize("小志向钱走一秒")
    payload = result.feedback_dict()

    assert payload["status"] == "normalized"
    assert payload["reason"] == "command_normalized"
    assert payload["original"] == "小志向钱走一秒"
    assert payload["normalized"] == "小智向前走一秒"
    assert payload["matches"]


def test_leaves_unrelated_sentences_unchanged():
    result = normalize("今天天气怎么样")

    assert result.text == "今天天气怎么样"
    assert not result.changed
    assert result.matches == []


def test_loads_external_rules_without_losing_builtin_defaults(tmp_path):
    config = tmp_path / "commands.yaml"
    config.write_text(
        """
aliases:
  - source: 往钱
    target: 往前
  - [兰灯, 蓝灯]
canonical_phrases:
  - 往前
  - 蓝灯
""",
        encoding="utf-8",
    )

    rules = NormalizationRules.from_yaml(config)
    normalizer = CommandNormalizer(rules=rules)

    assert normalizer.normalize("往钱走一秒").text == "往前走一秒"
    assert normalizer.normalize("兰灯").text == "蓝灯"
    # 内置词表仍然可用；外置配置是扩展，不是全量替换。
    assert normalizer.normalize("钱进一秒").text == "前进一秒"


def test_default_repo_rules_file_extends_builtin_rules():
    rules = NormalizationRules.from_yaml(
        PACKAGE_ROOT / "config" / "command_normalization_zh.yaml"
    )
    normalizer = CommandNormalizer(rules=rules)

    assert normalizer.normalize("往钱走一秒").text == "往前走一秒"
    assert normalizer.normalize("倒退一秒").text == "后退一秒"


def test_normalizes_real_microphone_navigation_cancel_typo():
    """锁定 180 秒真人录音中出现过的领域错词，避免再次漏掉取消指令。"""
    result = normalize("取消刀")

    assert result.text == "取消导航"
    actions = parse_fallback_action(result.text)
    assert actions.name == "cancel_navigation"
