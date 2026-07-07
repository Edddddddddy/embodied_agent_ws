from types import SimpleNamespace

import pytest

from embodied_offline_agent.providers.summer_tts_ros import (
    SummerTtsRosClient,
    SummerTtsRosError,
)


class FakeFuture:
    def __init__(self, response):
        self._response = response

    def done(self):
        return True

    def result(self):
        return self._response


class FakeClient:
    def __init__(self, response, available=True):
        self.response = response
        self.available = available
        self.requests = []

    def wait_for_service(self, timeout_sec):
        self.timeout_sec = timeout_sec
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.response)


class FakeNode:
    def __init__(self, client):
        self.client = client

    def create_client(self, _srv_type, service_name):
        self.service_name = service_name
        return self.client


def test_summer_tts_ros_provider_returns_pcm_and_updates_sample_rate():
    response = SimpleNamespace(
        ok=True,
        error="",
        sample_rate=16000,
        pcm=[1, 2, 3, 4],
    )
    client = FakeClient(response)
    provider = SummerTtsRosClient(
        FakeNode(client),
        service_name="/tts/synthesize",
        timeout_s=1.5,
        speaker_id=3,
        length_scale=1.2,
    )

    assert provider.synthesize("你好") == b"\x01\x02\x03\x04"
    assert provider.sample_rate == 16000
    request = client.requests[0]
    assert request.text == "你好"
    assert request.speaker_id == 3
    assert request.length_scale == pytest.approx(1.2)


def test_summer_tts_ros_provider_reports_unavailable_service():
    client = FakeClient(None, available=False)
    provider = SummerTtsRosClient(FakeNode(client), timeout_s=0.01)

    with pytest.raises(SummerTtsRosError, match="not available"):
        provider.synthesize("你好")


def test_summer_tts_ros_provider_surfaces_service_error():
    response = SimpleNamespace(ok=False, error="bad model", sample_rate=0, pcm=[])
    provider = SummerTtsRosClient(FakeNode(FakeClient(response)), timeout_s=0.01)

    with pytest.raises(SummerTtsRosError, match="bad model"):
        provider.synthesize("你好")
