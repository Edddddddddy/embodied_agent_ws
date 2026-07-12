from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import AgentTurnMetrics
from rclpy.qos import ReliabilityPolicy

from embodied_online_agent.agent_ros_io import AgentRosCallbacks, AgentRosIo
from embodied_online_agent.ros_topics import AgentTopicContract


class _Publisher:
    def __init__(self, topic, qos):
        self.topic = topic
        self.qos = qos
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Now:
    def to_msg(self):
        return Time(sec=1)


class _Clock:
    def now(self):
        return _Now()


class _Logger:
    def info(self, _message):
        pass


class _Node:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = []
        self.timers = []

    def create_lifecycle_publisher(self, _message_type, topic, qos):
        publisher = _Publisher(topic, qos)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = (message_type, topic, callback, qos)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period_s, callback):
        timer = (period_s, callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()


def _callbacks():
    callback = lambda _message: None
    return AgentRosCallbacks(
        text_input=callback,
        wake_event_input=callback,
        speaker_identity=callback,
        clear_memory=callback,
        action_result=callback,
        clean_audio=callback,
        silence_timeout=callback,
        speech_started=callback,
        speech_ended=callback,
    )


def test_ros_io_owns_common_topic_contract_without_optional_inputs():
    node = _Node()
    topics = AgentTopicContract()

    io = AgentRosIo(
        node,
        _callbacks(),
        microphone_enabled=False,
        external_wake_event_enabled=False,
        topics=topics,
    )

    subscribed_topics = {item[1] for item in node.subscriptions}
    assert subscribed_topics == {
        topics.text_input,
        topics.speaker_identity,
        topics.clear_memory,
        topics.action_result,
    }
    assert topics.metrics in node.publishers
    assert topics.clean_audio not in subscribed_topics
    io.publish_asr_final("向前走一秒")
    io.publish_tts_audio(b"\x01\x02")
    io.publish_metrics(AgentTurnMetrics(source="offline"))
    assert node.publishers[topics.asr_final].messages[-1].data == "向前走一秒"
    assert list(node.publishers[topics.tts_pcm].messages[-1].data) == [1, 2]
    assert node.publishers[topics.metrics].messages[-1].source == "offline"


def test_ros_io_applies_best_effort_only_to_realtime_audio():
    node = _Node()
    topics = AgentTopicContract()

    AgentRosIo(
        node,
        _callbacks(),
        microphone_enabled=True,
        external_wake_event_enabled=True,
        topics=topics,
    )

    subscriptions = {item[1]: item for item in node.subscriptions}
    assert topics.wake_event_input in subscriptions
    assert topics.speech_ended in subscriptions
    assert (
        subscriptions[topics.clean_audio][3].reliability
        == ReliabilityPolicy.BEST_EFFORT
    )
    assert (
        subscriptions[topics.text_input][3].reliability
        == ReliabilityPolicy.RELIABLE
    )
    assert (
        node.publishers[topics.tts_pcm].qos.reliability
        == ReliabilityPolicy.BEST_EFFORT
    )


def test_health_heartbeat_is_gated_by_agent_lifecycle():
    node = _Node()
    io = AgentRosIo(
        node,
        _callbacks(),
        microphone_enabled=False,
        external_wake_event_enabled=False,
    )
    health = node.publishers["system/component_health"]

    io.events.publish_ready("configured")
    assert health.messages == []

    io.set_lifecycle_active(True)
    io.events.publish_ready("active")
    assert health.messages[-1].detail == "active"

    io.events.publish_stopped("inactive")
    io.set_lifecycle_active(False)
    _, heartbeat = node.timers[0]
    heartbeat()
    assert len(health.messages) == 2
