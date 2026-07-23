from embodied_agent_core.asr_endpoint_runtime import AsrEndpointRuntime


class _FakeTimer:
    created = []

    def __init__(self, interval, callback, args=()):
        self.interval = interval
        self.callback = callback
        self.args = args
        self.daemon = False
        self.started = False
        self.cancelled = False
        self.created.append(self)

    def start(self):
        self.started = True

    def cancel(self):
        self.cancelled = True

    def fire(self):
        self.callback(*self.args)


def test_endpoint_is_deduplicated_and_delayed_before_commit():
    _FakeTimer.created.clear()
    now = [10.0]
    observed = []
    runtime = AsrEndpointRuntime(
        delay_ms=300,
        blocked=lambda: False,
        commit=lambda: observed.append("provider_commit"),
        on_endpoint=lambda source, delay: observed.append(("endpoint", source, delay)),
        on_commit=lambda source: observed.append(("commit", source)),
        on_duplicate=lambda source: observed.append(("duplicate", source)),
        clock=lambda: now[0],
        timer_factory=_FakeTimer,
    )

    assert runtime.request("speech_ended") is True
    assert runtime.request("silence_timeout") is False
    assert observed == [
        ("endpoint", "speech_ended", 300),
        ("duplicate", "silence_timeout"),
    ]
    assert _FakeTimer.created[0].interval == 0.3

    _FakeTimer.created[0].fire()
    assert observed[-2:] == [("commit", "speech_ended"), "provider_commit"]


def test_legacy_endpoint_is_still_deduplicated_after_dds_scheduling_jitter():
    _FakeTimer.created.clear()
    now = [10.0]
    duplicates = []
    runtime = AsrEndpointRuntime(
        delay_ms=450,
        blocked=lambda: False,
        commit=lambda: None,
        on_endpoint=lambda _source, _delay: None,
        on_commit=lambda _source: None,
        on_duplicate=duplicates.append,
        clock=lambda: now[0],
        timer_factory=_FakeTimer,
    )

    assert runtime.request("speech_ended") is True
    now[0] += 0.2
    assert runtime.request("silence_timeout") is False
    assert duplicates == ["silence_timeout"]


def test_same_endpoint_source_can_commit_consecutive_utterances():
    _FakeTimer.created.clear()
    now = [10.0]
    endpoints = []
    runtime = AsrEndpointRuntime(
        delay_ms=100,
        blocked=lambda: False,
        commit=lambda: None,
        on_endpoint=lambda source, _delay: endpoints.append(source),
        on_commit=lambda _source: None,
        clock=lambda: now[0],
        timer_factory=_FakeTimer,
    )

    assert runtime.request("speech_ended") is True
    now[0] += 0.12
    assert runtime.request("speech_ended") is True
    assert endpoints == ["speech_ended", "speech_ended"]


def test_busy_gate_and_close_prevent_late_provider_access():
    _FakeTimer.created.clear()
    blocked = [True]
    commits = []
    runtime = AsrEndpointRuntime(
        delay_ms=100,
        blocked=lambda: blocked[0],
        commit=lambda: commits.append("commit"),
        on_endpoint=lambda _source, _delay: None,
        on_commit=lambda _source: None,
        timer_factory=_FakeTimer,
    )

    assert runtime.request("speech_ended") is False
    blocked[0] = False
    assert runtime.request("speech_ended") is True
    runtime.close()
    assert _FakeTimer.created[0].cancelled is True
    _FakeTimer.created[0].fire()
    assert commits == []


def test_zero_delay_commits_synchronously():
    observed = []
    runtime = AsrEndpointRuntime(
        delay_ms=0,
        blocked=lambda: False,
        commit=lambda: observed.append("provider"),
        on_endpoint=lambda source, delay: observed.append((source, delay)),
        on_commit=lambda source: observed.append(source),
    )

    assert runtime.request("silence_timeout") is True
    assert observed == [("silence_timeout", 0), "silence_timeout", "provider"]


def test_provider_commit_failure_is_reported_without_escaping_timer_callback():
    errors = []

    def fail():
        raise RuntimeError("transport closed")

    runtime = AsrEndpointRuntime(
        delay_ms=0,
        blocked=lambda: False,
        commit=fail,
        on_endpoint=lambda _source, _delay: None,
        on_commit=lambda _source: None,
        on_error=errors.append,
    )

    assert runtime.request("speech_ended") is False
    assert len(errors) == 1
    assert str(errors[0]) == "transport closed"


def test_lifecycle_cancel_invalidates_old_timer_but_allows_next_activation():
    _FakeTimer.created.clear()
    commits = []
    runtime = AsrEndpointRuntime(
        delay_ms=100,
        blocked=lambda: False,
        commit=lambda: commits.append("commit"),
        on_endpoint=lambda _source, _delay: None,
        on_commit=lambda _source: None,
        timer_factory=_FakeTimer,
    )

    assert runtime.request("first_activation") is True
    old_timer = _FakeTimer.created[-1]
    runtime.cancel_pending()
    old_timer.fire()
    assert commits == []

    assert runtime.request("second_activation") is True
    _FakeTimer.created[-1].fire()
    assert commits == ["commit"]


def test_resumed_speech_keeps_buffered_prefix_and_commits_the_whole_utterance():
    _FakeTimer.created.clear()
    buffered_audio = ["向右转"]
    finals = []

    def commit_provider():
        finals.append("，".join(buffered_audio))
        buffered_audio.clear()

    runtime = AsrEndpointRuntime(
        delay_ms=300,
        blocked=lambda: False,
        commit=commit_provider,
        on_endpoint=lambda _source, _delay: None,
        on_commit=lambda _source: None,
        timer_factory=_FakeTimer,
    )

    assert runtime.request("speech_ended") is True
    first_timer = _FakeTimer.created[-1]
    runtime.resume_utterance()
    buffered_audio.append("向前走一秒")
    assert runtime.request("speech_ended") is True
    second_timer = _FakeTimer.created[-1]

    first_timer.fire()
    assert finals == []
    second_timer.fire()
    assert finals == ["向右转，向前走一秒"]
