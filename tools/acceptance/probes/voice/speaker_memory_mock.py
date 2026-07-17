import json
import os
import time
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import RobotCommand, SpeakerEnrollRequest, SpeakerIdentity
from embodied_agent_core.ros_qos import audio_qos, state_qos
from embodied_agent_core.ros_topics import AgentTopicContract
from embodied_agent_core.speaker_transport import identity_payload_to_message
from std_msgs.msg import String, UInt8MultiArray
from tools.acceptance.typed_action_probe_utils import candidate_dict


TOPICS = AgentTopicContract()


def _spin_until(node, predicate, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    memory_dir = Path(os.environ["USER_MEMORY_DIR"])
    rclpy.init()
    node = rclpy.create_node("speaker_memory_mock_test")
    responses = []
    actions = []
    enroll_requests = []
    tts_audio = []
    node.create_subscription(
        String, TOPICS.response_text, lambda msg: responses.append(msg.data), 10
    )
    node.create_subscription(
        RobotCommand,
        TOPICS.action_candidate,
        lambda msg: actions.append(candidate_dict(msg)),
        10,
    )
    node.create_subscription(
        SpeakerEnrollRequest,
        TOPICS.speaker_enroll_request,
        lambda msg: enroll_requests.append(
            {"speaker_id": msg.speaker_id, "display_name": msg.display_name}
        ),
        10,
    )
    node.create_subscription(
        UInt8MultiArray,
        TOPICS.tts_pcm,
        lambda msg: tts_audio.append(bytes(msg.data)),
        audio_qos(depth=20),
    )
    # 身份属于可回放状态，生产 Agent 以 transient-local 订阅；probe 必须使用
    # 同一 QoS，否则 DDS 会发现端点却拒绝匹配，现场表现为永久“未就绪”。
    identity_pub = node.create_publisher(
        SpeakerIdentity, TOPICS.speaker_identity, state_qos()
    )
    text_pub = node.create_publisher(String, TOPICS.text_input, 10)

    assert _spin_until(
        node,
        lambda: identity_pub.get_subscription_count() > 0
        and text_pub.get_subscription_count() > 0
        and node.count_publishers(TOPICS.tts_pcm) > 0,
        timeout_s=10.0,
    ), "agent subscriptions were not ready"

    identity_pub.publish(identity_payload_to_message({
        "speaker_id": "lcy",
        "display_name": "小李",
        "confidence": 0.93,
        "enrolled": True,
        "model": "mock-speaker",
    }))
    time.sleep(0.2)
    rclpy.spin_once(node, timeout_sec=0.2)

    tts_count = len(tts_audio)
    text_pub.publish(String(data="记住我，我是小李"))
    assert _spin_until(node, lambda: any("小李" in item for item in responses))
    assert _spin_until(
        node,
        lambda: any(item.get("speaker_id") == "lcy" for item in enroll_requests),
    )
    assert _spin_until(node, lambda: len(tts_audio) > tts_count)

    tts_count = len(tts_audio)
    text_pub.publish(String(data="我喜欢慢一点"))
    assert _spin_until(node, lambda: any("movement_speed=slow" in item for item in responses))
    assert _spin_until(node, lambda: len(tts_audio) > tts_count)

    tts_count = len(tts_audio)
    text_pub.publish(String(data="我是谁"))
    assert _spin_until(node, lambda: any("当前用户" in item and "小李" in item for item in responses))
    # 回复文本先发布、TTS 后台线程随后结束；等到音频可观测后再发下一句，
    # 才符合真人“听完回复继续说”的顺序，也避免把正常 busy 保护误判成丢命令。
    assert _spin_until(node, lambda: len(tts_audio) > tts_count)

    text_pub.publish(String(data="向前走"))
    assert _spin_until(node, lambda: any(item.get("name") == "move" for item in actions))
    assert any(
        payload.get("name") == "move"
        and payload.get("arguments", {}).get("linear_x") == 0.15
        for payload in actions
    ), "slow movement preference did not affect the next move command"
    profile_path = memory_dir / "lcy.json"
    assert _spin_until(
        node,
        lambda: profile_path.exists()
        and json.loads(profile_path.read_text(encoding="utf-8"))
        .get("command_counts", {})
        .get("move", 0)
        >= 1,
    )

    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    assert profile["display_name"] == "小李"
    assert profile["preferences"]["movement_speed"] == "slow"
    assert profile["command_counts"]["move"] >= 1

    response_count = len(responses)
    tts_count = len(tts_audio)
    text_pub.publish(String(data="我的偏好"))
    assert _spin_until(
        node,
        lambda: len(responses) > response_count
        and "movement_speed=slow" in responses[-1],
    )
    assert _spin_until(node, lambda: len(tts_audio) > tts_count)

    response_count = len(responses)
    tts_count = len(tts_audio)
    text_pub.publish(String(data="恢复默认速度"))
    assert _spin_until(
        node,
        lambda: len(responses) > response_count
        and "已删除偏好：movement_speed" in responses[-1],
    )
    assert _spin_until(node, lambda: len(tts_audio) > tts_count)
    assert _spin_until(
        node,
        lambda: "movement_speed"
        not in json.loads(profile_path.read_text(encoding="utf-8")).get(
            "preferences", {}
        ),
    )

    # clear 必须真的删除 profile；不能在回复后又把“清除记忆”作为 interaction 写回。
    response_count = len(responses)
    tts_count = len(tts_audio)
    text_pub.publish(String(data="清除我的记忆"))
    assert _spin_until(
        node,
        lambda: len(responses) > response_count and "已清除" in responses[-1],
    )
    assert _spin_until(node, lambda: len(tts_audio) > tts_count)
    assert _spin_until(node, lambda: not profile_path.exists())

    node.destroy_node()
    rclpy.shutdown()
    print(
        "PASS: speaker identity -> user memory -> query/delete/clear preference lifecycle"
    )


if __name__ == "__main__":
    main()
