from embodied_agent_core.user_memory import (
    LowConfidenceSpeakerError,
    SpeakerIdentity,
    UserMemoryStore,
    parse_memory_command,
)


def test_speaker_identity_rejects_low_confidence():
    identity = SpeakerIdentity(
        speaker_id="unknown", confidence=0.2, enrolled=False, model="mock"
    )

    assert identity.speaker_id == "unknown"
    assert not identity.usable


def test_user_memory_rejects_low_confidence_writes(tmp_path):
    store = UserMemoryStore(str(tmp_path), max_recent=2)
    identity = SpeakerIdentity(
        speaker_id="unknown", confidence=0.2, enrolled=False, model="mock"
    )

    assert store.prompt_summary(identity) == ""
    for operation in (
        lambda: store.enroll(identity, "小李"),
        lambda: store.set_preference(identity, "movement_speed", "slow"),
        lambda: store.remove_preference(identity, "movement_speed"),
        lambda: store.record_interaction(identity, user_text="向前走"),
        lambda: store.clear(identity),
    ):
        try:
            operation()
        except LowConfidenceSpeakerError as error:
            assert "low confidence" in str(error)
        else:
            raise AssertionError("low-confidence speaker write was accepted")

    assert not list(tmp_path.glob("*.json"))


def test_user_memory_enroll_preference_and_prompt_summary(tmp_path):
    store = UserMemoryStore(str(tmp_path), max_recent=2)
    identity = SpeakerIdentity("lcy", confidence=0.92, enrolled=True, model="mock")

    store.enroll(identity, "小李")
    store.set_preference(identity, "movement_speed", "slow")
    store.record_interaction(
        identity,
        user_text="向前走",
        assistant_text="好的",
        actions=[{"name": "move", "arguments": {"duration_s": 1.0}}],
        success=True,
    )

    loaded = store.profile(identity)
    assert loaded.display_name == "小李"
    assert loaded.preferences["movement_speed"] == "slow"
    assert loaded.command_counts["move"] == 1
    summary = store.prompt_summary(identity)
    assert "当前识别用户：小李" in summary
    assert "movement_speed=slow" in summary
    assert "move×1" in summary


def test_user_memory_bounds_recent_interactions(tmp_path):
    store = UserMemoryStore(str(tmp_path), max_recent=2)
    identity = SpeakerIdentity("lcy", confidence=1.0, enrolled=True, model="mock")

    for index in range(3):
        store.record_interaction(identity, user_text=f"cmd{index}")

    assert [item["user"] for item in store.profile(identity).recent_interactions] == [
        "cmd1",
        "cmd2",
    ]


def test_user_memory_expires_interaction_details_but_keeps_explicit_preferences(tmp_path):
    now = [1000.0]
    store = UserMemoryStore(
        str(tmp_path), retention_s=60.0, clock=lambda: now[0]
    )
    identity = SpeakerIdentity("lcy", confidence=1.0, enrolled=True, model="mock")
    store.set_preference(identity, "movement_speed", "slow")
    store.record_interaction(identity, user_text="隐私话术")

    now[0] += 61.0
    profile = store.profile(identity)
    assert profile.recent_interactions == []
    assert profile.preferences == {"movement_speed": "slow"}


def test_user_memory_removes_one_preference_without_deleting_profile(tmp_path):
    store = UserMemoryStore(str(tmp_path))
    identity = SpeakerIdentity("lcy", confidence=1.0, enrolled=True, model="mock")
    store.set_preference(identity, "movement_speed", "slow")
    store.set_preference(identity, "default_move_duration_s", 2.0)

    profile = store.remove_preference(identity, "movement_speed")
    assert profile.preferences == {"default_move_duration_s": 2.0}


def test_parse_memory_management_commands():
    assert parse_memory_command("记住我，我是小李").kind == "enroll_name"
    assert parse_memory_command("我是谁").kind == "whoami"
    assert parse_memory_command("清除我的记忆").kind == "clear"
    assert parse_memory_command("我的偏好").kind == "query_preferences"
    assert parse_memory_command("恢复默认速度").value == "movement_speed"
    assert parse_memory_command("取消默认前进时长").value == "default_move_duration_s"
    assert parse_memory_command("删除默认转弯角度").value == "default_turn_degrees"
    assert parse_memory_command("我喜欢慢一点").value == {"movement_speed": "slow"}
    assert parse_memory_command("以后前进默认两秒").value == {
        "default_move_duration_s": 2.0
    }
    assert parse_memory_command("以后默认左转四十五度").value == {
        "default_turn_degrees": 45.0
    }
    assert parse_memory_command("以后转弯默认90度").value == {
        "default_turn_degrees": 90.0
    }
