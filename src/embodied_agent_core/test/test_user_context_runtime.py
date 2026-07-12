from embodied_agent_core.user_context_runtime import UserContextRuntime
from embodied_agent_core.user_memory import SpeakerIdentity, UserMemoryStore


def _identity(name: str) -> SpeakerIdentity:
    return SpeakerIdentity(
        speaker_id=name,
        confidence=0.95,
        enrolled=True,
        model="test",
        display_name=name,
    )


def test_unknown_speaker_gets_empty_read_context_and_cannot_write(tmp_path):
    runtime = UserContextRuntime(UserMemoryStore(str(tmp_path)))

    context = runtime.snapshot()

    assert dict(context.preferences) == {}
    assert context.prompt_summary == ""
    assert context.system_prompt("base") == "base"
    assert runtime.record_interaction(
        context,
        user_text="前进",
        assistant_text="好的",
        actions=[],
        success=True,
    ) is False
    assert list(tmp_path.glob("*.json")) == []


def test_snapshot_keeps_identity_and_preferences_stable_across_speaker_change(tmp_path):
    store = UserMemoryStore(str(tmp_path))
    alice = _identity("alice")
    bob = _identity("bob")
    store.set_preference(alice, "movement_speed", "slow")
    store.set_preference(bob, "movement_speed", "fast")
    runtime = UserContextRuntime(store)
    runtime.update_identity(alice)

    alice_context = runtime.snapshot()
    runtime.update_identity(bob)
    stored = runtime.record_interaction(
        alice_context,
        user_text="前进",
        assistant_text="好的",
        actions=[{"name": "move", "arguments": {}}],
        success=True,
    )

    assert stored is True
    assert alice_context.identity.speaker_id == "alice"
    assert alice_context.preferences["movement_speed"] == "slow"
    assert store.profile(alice).command_counts == {"move": 1}
    assert store.profile(bob).command_counts == {}


def test_memory_command_can_enroll_identity_then_snapshot_prompt(tmp_path):
    runtime = UserContextRuntime(UserMemoryStore(str(tmp_path)))

    result = runtime.handle_command("记住我，我是小李")
    context = runtime.snapshot()

    assert result is not None
    assert runtime.identity.speaker_id == "小李"
    assert context.identity.speaker_id == "小李"
    assert "当前识别用户：小李" in context.system_prompt("base")


def test_snapshot_preferences_are_read_only(tmp_path):
    store = UserMemoryStore(str(tmp_path))
    runtime = UserContextRuntime(store)
    runtime.update_identity(_identity("alice"))
    context = runtime.snapshot()

    try:
        context.preferences["movement_speed"] = "fast"
    except TypeError:
        pass
    else:
        raise AssertionError("snapshot preferences must be immutable")
