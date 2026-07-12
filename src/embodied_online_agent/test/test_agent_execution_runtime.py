import threading
import time

from embodied_online_agent.agent_control_plane import (
    AgentControlPlane,
    AgentControlPlaneConfig,
)
from embodied_online_agent.agent_execution_runtime import AgentExecutionRuntime


def _control(*, continuous: bool = True) -> AgentControlPlane:
    return AgentControlPlane(
        AgentControlPlaneConfig(
            source="test",
            continuous_enabled=continuous,
            queue_size=4,
            command_max_age_s=30.0,
            wake_words=["小智"],
            wake_word_aliases=[],
            wake_word_enabled=False,
            wake_timeout_s=60.0,
            duplicate_window_s=0.0,
            recognition_max_retries=1,
            command_normalization_enabled=False,
            normalization_feedback_enabled=False,
            normalization_fuzzy_threshold=0.8,
            normalization_rules_path="",
            command_completion_enabled=False,
            command_nlu_enabled=False,
            command_nlu_min_confidence=0.2,
            partial_merge_enabled=False,
            partial_max_age_s=2.0,
        )
    )


def test_worker_isolates_one_failure_and_resets_busy_after_preparsed_turn():
    control = _control()
    execution_events = []
    errors = []
    finished = threading.Event()
    wait_policy = []

    def execute(item):
        wait_policy.append(runtime.should_wait_for_action_results(1))
        if item.text == "bad":
            raise RuntimeError("broken command")
        finished.set()

    runtime = AgentExecutionRuntime(
        control,
        execute_item=execute,
        publish_execution=execution_events.append,
        publish_queue=lambda _event: None,
        on_error=errors.append,
    )
    control.command_queue.put("bad")
    control.command_queue.put("good", context={"preparsed_actions": []})
    runtime.start()

    assert finished.wait(timeout=1.0)
    deadline = time.monotonic() + 1.0
    while runtime.is_busy() and time.monotonic() < deadline:
        time.sleep(0.01)
    runtime.stop()

    finished_events = [event for event in execution_events if event.event == "finished"]
    assert [event.success for event in finished_events] == [False, True]
    assert "broken command" in finished_events[0].reason
    assert len(errors) == 1
    assert wait_policy == [True, True]
    assert runtime.is_busy() is False


def test_non_continuous_turn_gate_is_atomic_and_multi_action_always_waits():
    control = _control(continuous=False)
    runtime = AgentExecutionRuntime(
        control,
        execute_item=lambda _item: None,
        publish_execution=lambda _event: None,
        publish_queue=lambda _event: None,
        on_error=lambda _error: None,
    )

    assert runtime.try_begin_turn() is True
    assert runtime.try_begin_turn() is False
    assert runtime.should_wait_for_action_results(2) is True
    runtime.finish_turn()
    assert runtime.try_begin_turn() is True
    runtime.finish_turn()


def test_background_turn_is_owned_cancelled_and_restartable():
    control = _control(continuous=False)
    entered = threading.Event()
    cancelled = threading.Event()
    errors = []

    def execute():
        entered.set()
        try:
            while True:
                runtime.raise_if_stopping()
                time.sleep(0.005)
        finally:
            cancelled.set()

    runtime = AgentExecutionRuntime(
        control,
        execute_item=lambda _item: None,
        publish_execution=lambda _event: None,
        publish_queue=lambda _event: None,
        on_error=errors.append,
    )
    assert runtime.start() is True
    assert runtime.start_background_turn(execute) is True
    assert entered.wait(timeout=1.0)

    assert runtime.stop(timeout_s=1.0) is True
    assert cancelled.is_set()
    assert runtime.is_busy() is False
    assert errors == []

    completed = threading.Event()
    assert runtime.start() is True
    assert runtime.start_background_turn(completed.set) is True
    assert completed.wait(timeout=1.0)
    deadline = time.monotonic() + 1.0
    while runtime.is_busy() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert runtime.is_busy() is False
    assert runtime.stop() is True


def test_stop_reports_non_cooperative_turn_instead_of_starting_a_second_owner():
    control = _control(continuous=False)
    release = threading.Event()
    entered = threading.Event()
    runtime = AgentExecutionRuntime(
        control,
        execute_item=lambda _item: None,
        publish_execution=lambda _event: None,
        publish_queue=lambda _event: None,
        on_error=lambda _error: None,
    )
    runtime.start()

    def execute():
        entered.set()
        release.wait(timeout=1.0)

    assert runtime.start_background_turn(execute) is True
    assert entered.wait(timeout=1.0)
    assert runtime.stop(timeout_s=0.01) is False
    assert runtime.start() is False
    release.set()
    assert runtime.stop(timeout_s=1.0) is True


def test_continuous_worker_waits_for_managed_background_turn():
    control = _control(continuous=True)
    background_entered = threading.Event()
    release_background = threading.Event()
    command_executed = threading.Event()
    runtime = AgentExecutionRuntime(
        control,
        execute_item=lambda _item: command_executed.set(),
        publish_execution=lambda _event: None,
        publish_queue=lambda _event: None,
        on_error=lambda _error: None,
    )
    runtime.start()

    def background():
        background_entered.set()
        release_background.wait(timeout=1.0)

    assert runtime.start_background_turn(background) is True
    assert background_entered.wait(timeout=1.0)
    control.command_queue.put("queued-after-memory-response")
    time.sleep(0.05)
    assert command_executed.is_set() is False

    release_background.set()
    assert command_executed.wait(timeout=1.0)
    assert runtime.stop() is True
