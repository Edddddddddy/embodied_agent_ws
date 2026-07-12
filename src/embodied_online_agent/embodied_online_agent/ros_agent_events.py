"""Agent 控制面领域事件到 ROS 2 typed topic 的唯一 Adapter。"""

from embodied_agent_interfaces.msg import (
    ComponentHealth,
    CommandExecutionEvent,
    CommandQueueEvent,
    NluParseEvent,
    RecognitionFeedback,
    WakeEvent,
)
from rclpy.node import Node
from std_msgs.msg import String

from .agent_control_plane import TranscriptControlDecision
from .continuous_voice import QueueSnapshot
from .ros_event_transport import (
    execution_event_to_message,
    nlu_parse_to_message,
    queue_event_to_message,
    recognition_feedback_to_message,
    wake_event_to_message,
)
from .ros_qos import command_event_qos, latched_state_qos


class RosAgentEventPublisher:
    """统一控制面 topic、时间戳和 QoS，节点只表达业务事件。"""

    def __init__(self, node: Node, component_name: str = "agent"):
        self._node = node
        self._component_name = component_name
        self._state = node.create_publisher(
            String, "/agent/state", latched_state_qos()
        )
        self._wake = node.create_publisher(
            WakeEvent, "/agent/wake_event", command_event_qos()
        )
        self._session = node.create_publisher(
            String, "/agent/session_state", latched_state_qos()
        )
        self._queue = node.create_publisher(
            CommandQueueEvent, "/agent/command_queue", command_event_qos()
        )
        self._execution = node.create_publisher(
            CommandExecutionEvent,
            "/agent/command_execution",
            command_event_qos(),
        )
        self._recognition = node.create_publisher(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            command_event_qos(),
        )
        self._nlu = node.create_publisher(
            NluParseEvent, "/agent/nlu_parse", command_event_qos()
        )
        self._health = node.create_publisher(
            ComponentHealth, "system/component_health", latched_state_qos()
        )
        self._health_detail = None
        self._health_timer = node.create_timer(1.0, self._publish_health_heartbeat)

    def _stamp(self):
        return self._node.get_clock().now().to_msg()

    def publish_state(self, state: str) -> None:
        self._state.publish(String(data=state))

    def publish_ready(self, detail: str) -> None:
        """Agent provider 完成创建/预热后发布统一组件就绪状态。"""
        self._health_detail = detail
        self._publish_health_heartbeat()

    def _publish_health_heartbeat(self) -> None:
        if self._health_detail is None:
            return
        message = ComponentHealth()
        message.stamp = self._stamp()
        message.component = self._component_name
        message.state = ComponentHealth.STATE_READY
        message.detail = self._health_detail
        self._health.publish(message)

    def publish_session_event(self, event) -> None:
        self._wake.publish(
            wake_event_to_message(event.wake_event, stamp=self._stamp())
        )
        self._session.publish(String(data=event.session_state))

    def publish_queue(self, payload) -> None:
        self._queue.publish(queue_event_to_message(payload, stamp=self._stamp()))

    def publish_execution(self, event) -> None:
        self._execution.publish(
            execution_event_to_message(event, stamp=self._stamp())
        )

    def publish_recognition(self, payload: dict) -> None:
        self._recognition.publish(
            recognition_feedback_to_message(payload, stamp=self._stamp())
        )

    def publish_nlu(
        self, transcript: str, nlu_result, batch_id: str, *, source: str
    ) -> None:
        self._nlu.publish(
            nlu_parse_to_message(
                transcript,
                nlu_result,
                batch_id,
                source=source,
                stamp=self._stamp(),
            )
        )

    def publish_ignored(self, transcript: str, reason: str) -> None:
        self.publish_recognition(
            {"status": "ignored", "reason": reason, "transcript": transcript}
        )
        self._node.get_logger().info(
            f"ignored ASR final: reason={reason}, text={transcript}"
        )

    def publish_queue_rejected(
        self, transcript: str, snapshot: QueueSnapshot
    ) -> None:
        self.publish_recognition(
            {
                "status": "queue_rejected",
                "reason": snapshot.reason,
                "transcript": transcript,
                "queue_size": snapshot.size,
            }
        )

    def publish_asr_endpoint(self, source: str, delay_ms: int) -> None:
        self.publish_recognition(
            {"status": "asr_endpoint", "source": source, "delay_ms": delay_ms}
        )

    def publish_asr_commit(self, source: str) -> None:
        self.publish_recognition({"status": "asr_commit", "source": source})

    def publish_session_timeout(self, transcript: str) -> None:
        self.publish_recognition(
            {
                "status": "session_timeout",
                "reason": "voice_session_timeout",
                "transcript": transcript,
                "prompt": "会话已超时，请先说小智",
            }
        )

    def publish_control_decision(
        self, decision: TranscriptControlDecision
    ) -> None:
        """按统一顺序发布一次控制面决策产生的全部观测事件。"""
        if decision.session_event is not None:
            self.publish_session_event(decision.session_event)
        for payload in decision.recognition_feedback:
            self.publish_recognition(payload)
            if payload.get("status") == "ignored":
                self._node.get_logger().info(
                    "ignored ASR final: "
                    f"reason={payload.get('reason')}, text={decision.transcript}"
                )
        if decision.queue_event is not None:
            self.publish_queue(decision.queue_event)
        if decision.state:
            self.publish_state(decision.state)
