from types import SimpleNamespace

from embodied_offline_agent.offline_agent_node import OfflineAgentNode


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


def _node(*, speech_endpoint_events_enabled=True):
    node = object.__new__(OfflineAgentNode)
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

    node._on_silence(None)

    assert endpoint.requests == []


def test_legacy_silence_remains_the_fallback_when_speech_endpoints_are_disabled():
    node, endpoint, _, _ = _node(speech_endpoint_events_enabled=False)

    node._on_silence(None)

    assert endpoint.requests == ["silence_timeout"]
