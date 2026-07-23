from types import SimpleNamespace

from embodied_online_agent.online_agent_node import OnlineAgentNode


class _Endpoint:
    def __init__(self):
        self.cancel_count = 0
        self.requests = []

    def resume_utterance(self):
        self.cancel_count += 1

    def request(self, source):
        self.requests.append(source)


class _Stabilizer:
    def __init__(self):
        self.clear_count = 0

    def clear(self):
        self.clear_count += 1


class _Execution:
    def __init__(self):
        self.checks = 0

    def raise_if_stopping(self):
        self.checks += 1


class _Tts:
    def __init__(self):
        self.requests = []

    def synthesize(self, chunks, on_audio):
        self.requests.append(list(chunks))
        on_audio(b"memory-pcm")


class _RosIo:
    def __init__(self):
        self.audio = []

    def publish_tts_audio(self, pcm):
        self.audio.append(pcm)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def _node(*, speech_endpoint_events_enabled=True):
    node = object.__new__(OnlineAgentNode)
    endpoint = _Endpoint()
    stabilizer = _Stabilizer()
    node._runtime = SimpleNamespace(active=True, endpoint=endpoint, execution=None)
    node._continuous_enabled = True
    node._control = SimpleNamespace(transcript_stabilizer=stabilizer)
    node._agent_parameters = {
        "speech_endpoint_events_enabled": speech_endpoint_events_enabled,
    }
    states = []
    node._publish_state = states.append
    return node, endpoint, stabilizer, states


def test_speech_started_cancels_pending_endpoint_before_clearing_partial():
    node, endpoint, stabilizer, states = _node()

    node._on_speech_started(None)

    assert endpoint.cancel_count == 1
    assert stabilizer.clear_count == 1
    assert states == ["speech_detected"]


def test_legacy_silence_is_ignored_when_speech_endpoint_events_are_enabled():
    node, endpoint, _, _ = _node(speech_endpoint_events_enabled=True)

    node._on_silence_timeout(None)

    assert endpoint.requests == []


def test_legacy_silence_remains_the_fallback_when_speech_endpoints_are_disabled():
    node, endpoint, _, _ = _node(speech_endpoint_events_enabled=False)

    node._on_silence_timeout(None)

    assert endpoint.requests == ["silence_timeout"]


def test_memory_response_uses_ros_io_without_turn_private_callback():
    """记忆回复有独立 TTS 路径，不能依赖流式 turn 的私有实现。"""

    node = object.__new__(OnlineAgentNode)
    execution = _Execution()
    node._runtime = SimpleNamespace(execution=execution)
    node.tts = _Tts()
    node._ros_io = _RosIo()
    logger = _Logger()
    node.get_logger = lambda: logger

    node._speak_memory_response("已记住你的偏好")

    assert execution.checks == 2
    assert node.tts.requests == [["已记住你的偏好"]]
    assert node._ros_io.audio == [b"memory-pcm"]
    assert logger.warnings == []
