from embodied_online_agent.memory_command_service import MemoryCommandService
from embodied_online_agent.user_memory import SpeakerIdentity, UserMemoryStore


def _identity(name="alice"):
    return SpeakerIdentity(
        speaker_id=name,
        confidence=0.9,
        enrolled=True,
        model="test",
        display_name=name.title(),
    )


def test_memory_command_service_owns_preference_lifecycle(tmp_path):
    store = UserMemoryStore(str(tmp_path), clock=lambda: 100.0)
    service = MemoryCommandService(store, clock=lambda: 100.0)
    identity = _identity()

    saved = service.handle("记住我喜欢慢一点", identity)
    queried = service.handle("我的偏好是什么", identity)
    deleted = service.handle("删除移动速度偏好", identity)

    assert "movement_speed=slow" in saved.response
    assert "movement_speed=slow" in queried.response
    assert "已删除偏好" in deleted.response
    assert store.profile(identity).preferences == {}
    assert store.profile(identity).recent_interactions


def test_memory_command_service_returns_enrollment_as_data(tmp_path):
    service = MemoryCommandService(
        UserMemoryStore(str(tmp_path), clock=lambda: 123.0),
        clock=lambda: 123.0,
    )

    result = service.handle("记住我，我是小王", SpeakerIdentity())

    assert result.identity.speaker_id == "小王"
    assert result.identity.model == "text-enroll-fallback"
    assert result.enroll_request.speaker_id == "小王"
    assert result.enroll_request.samples_required == 3


def test_unknown_user_query_does_not_create_unknown_profile(tmp_path):
    store = UserMemoryStore(str(tmp_path))
    result = MemoryCommandService(store).handle("我是谁", SpeakerIdentity())

    assert "还没有可靠识别" in result.response
    assert list(tmp_path.iterdir()) == []


def test_non_memory_command_is_not_consumed(tmp_path):
    service = MemoryCommandService(UserMemoryStore(str(tmp_path)))
    assert service.handle("向前走一秒", _identity()) is None


def test_auxiliary_interaction_write_failure_does_not_break_reply(tmp_path, monkeypatch):
    store = UserMemoryStore(str(tmp_path))
    monkeypatch.setattr(
        store,
        "record_interaction",
        lambda *args, **kwargs: (_ for _ in ()).throw(OSError("disk unavailable")),
    )

    result = MemoryCommandService(store).handle("我是谁", _identity())

    assert "当前用户" in result.response
