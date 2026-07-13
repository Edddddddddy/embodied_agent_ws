from dataclasses import dataclass
from types import SimpleNamespace

from embodied_agent_core.action_sequence import SequencePublishReport
from embodied_agent_core.agent_application_runtime import (
    AgentApplicationCallbacks,
    AgentApplicationRuntime,
)
from embodied_agent_core.types import ActionCommand
from embodied_agent_core.user_context_runtime import UserContextSnapshot
from embodied_agent_core.user_memory import SpeakerIdentity


class FakeLogger:
    def __init__(self):
        self.messages = []

    def __getattr__(self, level):
        return lambda message: self.messages.append((level, message))


class FakeEvents:
    def __init__(self):
        self.states = []
        self.decisions = []
        self.enqueues = []
        self.sessions = []
        self.queues = []

    def publish_state(self, state):
        self.states.append(state)

    def publish_control_decision(self, decision):
        self.decisions.append(decision)

    def publish_enqueue_decision(self, decision):
        self.enqueues.append(decision)

    def publish_session_event(self, event):
        self.sessions.append(event)

    def publish_queue(self, event):
        self.queues.append(event)


class FakeExecution:
    def __init__(self, worker=False):
        self.started = []
        self.worker = worker

    def start_background_turn(self, callback, *args):
        self.started.append((callback, args))
        return True

    def should_wait_for_action_results(self, count):
        return count > 1 or (count > 0 and self.worker)


class FakeLifecycle:
    def __init__(self, execution=None):
        self.execution = execution


class FakeSequencer:
    def __init__(self):
        self.published = []
        self.cancels = []
        self.results = []

    def publish(self, actions, publish, *, wait_for_results):
        actions = list(actions)
        for action in actions:
            publish(action)
        self.published.append((actions, wait_for_results))
        return SequencePublishReport(len(actions), len(actions), False)

    def cancel(self, reason):
        self.cancels.append(reason)

    def notify_result(self, command_id, success, message, *, status=0):
        self.results.append((command_id, success, message, status))


class FakeMemory:
    def __init__(self):
        self.cleared = 0

    def clear(self):
        self.cleared += 1


class FakeUserContext:
    def __init__(self, memory_result=None):
        self.memory_result = memory_result
        self.identity = SpeakerIdentity()
        self.recorded = []
        self.cleared = 0

    def snapshot(self):
        return UserContextSnapshot(self.identity, {})

    def handle_command(self, _command):
        return self.memory_result

    def update_identity(self, identity):
        self.identity = identity

    def clear_current(self):
        self.cleared += 1
        return True

    def record_interaction(self, context, **kwargs):
        self.recorded.append((context, kwargs))
        return True


@dataclass
class Decision:
    directive: str
    command: str = ""
    cancel_reason: str = ""
    priority_action: str = ""
    dropped: int = 0


class FakeControl:
    continuous_enabled = False

    def __init__(self, decision):
        self.decision = decision

    def accept_transcript(self, transcript, *, wake_word_required):
        self.accepted = (transcript, wake_word_required)
        return self.decision

    def enqueue_command(self, command, *, context_extras, fallback_context):
        self.enqueued = (command, context_extras, fallback_context)
        return SimpleNamespace(
            status="queued", queue_size=1, reason="", source_text=command
        )

    @staticmethod
    def preparsed_actions(context):
        return tuple(context.get("preparsed_actions", ()))


class Item:
    def __init__(self, text, context):
        self.text = text
        self.context = context


def make_runtime(*, decision=Decision("noop"), execution=None, continuous=False):
    control = FakeControl(decision)
    control.continuous_enabled = continuous
    events = FakeEvents()
    sequencer = FakeSequencer()
    user_context = FakeUserContext()
    memory = FakeMemory()
    calls = {
        "model": [],
        "prepared": [],
        "finished": [],
        "actions": [],
        "responses": [],
    }
    callbacks = AgentApplicationCallbacks(
        run_model_turn=lambda text, ctx, user: calls["model"].append((text, ctx, user)),
        speak_memory_response=lambda text: calls.setdefault("spoken", []).append(text),
        publish_action_candidate=lambda action: calls["actions"].append(action),
        publish_enroll_request=lambda request: calls.setdefault("enroll", []).append(request),
        publish_response_delta=lambda text: calls["responses"].append(("delta", text)),
        publish_response=lambda text: calls["responses"].append(("final", text)),
        capture_turn_context=lambda: "captured",
        prepare_turn_context=lambda ctx: calls["prepared"].append(ctx),
        finish_preparsed_turn=lambda ctx: calls["finished"].append(ctx),
    )
    runtime = AgentApplicationRuntime(
        source="test",
        control=control,
        lifecycle_runtime=FakeLifecycle(execution),
        action_sequencer=sequencer,
        conversation_memory=memory,
        user_context=user_context,
        events=events,
        callbacks=callbacks,
        logger=FakeLogger(),
        wake_word_required=True,
        continuous_enabled=continuous,
    )
    return runtime, control, events, sequencer, user_context, memory, calls


def test_direct_transcript_freezes_context_and_starts_managed_turn():
    execution = FakeExecution()
    runtime, control, _, _, _, _, calls = make_runtime(
        decision=Decision("command", command="前进一秒"), execution=execution
    )

    runtime.accept_transcript("小智前进一秒")

    assert control.accepted == ("小智前进一秒", True)
    callback, args = execution.started[0]
    callback(*args)
    assert calls["prepared"] == ["captured"]
    assert calls["model"][0][:2] == ("前进一秒", "captured")


def test_priority_command_cancels_sequence_and_publishes_stop():
    runtime, _, _, sequencer, _, _, calls = make_runtime(
        decision=Decision(
            "priority", cancel_reason="priority_stop", priority_action="stop", dropped=2
        )
    )

    runtime.accept_transcript("停下")

    assert sequencer.cancels == ["priority_stop"]
    assert calls["actions"] == [ActionCommand("stop", {}, priority=True)]


def test_continuous_command_freezes_user_and_turn_context_before_enqueue():
    runtime, control, events, _, _, _, _ = make_runtime(
        decision=Decision("command", command="前进一秒"),
        execution=FakeExecution(worker=True),
        continuous=True,
    )

    runtime.accept_transcript("前进一秒")

    command, extras, fallback = control.enqueued
    assert command == "前进一秒"
    assert extras is fallback
    assert extras["turn_context"] == "captured"
    assert isinstance(extras["user_context"], UserContextSnapshot)
    assert len(events.enqueues) == 1


def test_queued_preparsed_turn_uses_frozen_context_and_finishes_metrics():
    execution = FakeExecution(worker=True)
    runtime, _, events, sequencer, user_context, _, calls = make_runtime(
        execution=execution, continuous=True
    )
    snapshot = user_context.snapshot()
    action = ActionCommand("turn", {"angular_z": 0.5, "duration_s": 1.0})

    runtime.run_queued_turn(
        Item(
            "左转",
            {
                "turn_context": "latency-1",
                "user_context": snapshot,
                "preparsed_actions": [action],
            },
        )
    )

    assert calls["prepared"] == ["latency-1"]
    assert calls["finished"] == ["latency-1"]
    assert sequencer.published[0][1] is True
    assert events.states == ["thinking"]
    assert user_context.recorded[0][1]["success"] is True


def test_clear_memory_clears_conversation_and_current_profile():
    runtime, _, _, _, user_context, memory, _ = make_runtime()

    runtime.clear_memory()

    assert memory.cleared == 1
    assert user_context.cleared == 1
