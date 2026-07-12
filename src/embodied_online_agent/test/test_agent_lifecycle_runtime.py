import pytest

from embodied_online_agent.agent_lifecycle_runtime import AgentLifecycleRuntime


class _Queue:
    def __init__(self, calls):
        self.calls = calls

    def clear(self):
        self.calls.append("queue.clear")


class _Control:
    def __init__(self, calls):
        self.calls = calls
        self.command_queue = _Queue(calls)

    def reset_session(self, source):
        self.calls.append(f"session.reset:{source}")
        return "session-event"


class _Sequencer:
    def __init__(self, calls):
        self.calls = calls

    def cancel(self, reason):
        self.calls.append(f"sequence.cancel:{reason}")


class _RosIo:
    def __init__(self, calls):
        self.calls = calls

    def set_lifecycle_active(self, active):
        self.calls.append(f"ros.active:{active}")


class _Events:
    def __init__(self, calls):
        self.calls = calls

    def publish_session_event(self, event):
        self.calls.append(f"session.publish:{event}")

    def publish_state(self, state):
        self.calls.append(f"state:{state}")

    def publish_stopped(self, detail):
        self.calls.append(f"stopped:{detail}")


class _Execution:
    def __init__(self, calls, *, start=True, stop=True):
        self.calls = calls
        self.start_result = start
        self.stop_result = stop

    def start(self):
        self.calls.append("execution.start")
        return self.start_result

    def stop(self, timeout):
        self.calls.append(f"execution.stop:{timeout}")
        return self.stop_result


class _Endpoint:
    def __init__(self, calls):
        self.calls = calls

    def cancel_pending(self):
        self.calls.append("endpoint.cancel")

    def close(self):
        self.calls.append("endpoint.close")


def _runtime(calls, *, execution_stop=True):
    runtime = AgentLifecycleRuntime(
        control=_Control(calls),
        action_sequencer=_Sequencer(calls),
        ros_io=_RosIo(calls),
        events=_Events(calls),
        publish_priority_stop=lambda: calls.append("action.stop"),
        deactivate_timeout_s=2.5,
    )
    runtime.bind(
        execution=_Execution(calls, stop=execution_stop),
        endpoint=_Endpoint(calls),
    )
    return runtime


def test_activate_requires_bound_resources_and_opens_ros_before_input():
    calls = []
    runtime = AgentLifecycleRuntime(
        control=_Control(calls),
        action_sequencer=_Sequencer(calls),
        ros_io=_RosIo(calls),
        events=_Events(calls),
        publish_priority_stop=lambda: calls.append("action.stop"),
        deactivate_timeout_s=1.0,
    )
    with pytest.raises(RuntimeError, match="not configured"):
        runtime.activate(lambda: None)

    runtime.bind(execution=_Execution(calls), endpoint=_Endpoint(calls))
    runtime.activate(lambda: calls.append("input.start"))
    assert runtime.active is True
    assert calls[-3:] == ["ros.active:True", "input.start", "execution.start"]


def test_deactivate_enforces_safe_stop_order_and_closes_ros_last():
    calls = []
    runtime = _runtime(calls)
    runtime.activate(lambda: calls.append("input.start"))
    calls.clear()

    result = runtime.deactivate(lambda: calls.append("input.stop") or True)

    assert result.quiesced is True
    assert calls == [
        "endpoint.cancel",
        "sequence.cancel:lifecycle_deactivated",
        "queue.clear",
        "session.reset:lifecycle",
        "session.publish:session-event",
        "action.stop",
        "input.stop",
        "execution.stop:2.5",
        "state:inactive",
        "stopped:lifecycle_inactive",
        "ros.active:False",
    ]


def test_deactivate_timeout_keeps_publishers_available_for_error_observation():
    calls = []
    runtime = _runtime(calls, execution_stop=False)
    runtime.activate(lambda: None)
    calls.clear()

    result = runtime.deactivate(lambda: True)

    assert result.quiesced is False
    assert "state:deactivate_timeout" in calls
    assert "stopped:deactivate_timeout" in calls
    assert "ros.active:False" not in calls


def test_release_unbinds_only_after_all_workers_quiesce():
    calls = []
    runtime = _runtime(calls)

    result = runtime.release(lambda: True)

    assert result.quiesced is True
    assert runtime.configured is False
    assert calls[-3:] == [
        "ros.active:False",
        "endpoint.close",
        "execution.stop:2.5",
    ]


def test_shutdown_is_idempotent_and_publishes_stop_only_when_active():
    calls = []
    runtime = _runtime(calls)
    runtime.activate(lambda: None)
    calls.clear()

    assert runtime.begin_shutdown() is True
    assert runtime.begin_shutdown() is False
    assert calls.count("action.stop") == 1
    assert runtime.stopping is True
