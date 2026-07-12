from embodied_online_agent.action_sequence import SequentialActionPublisher
from embodied_online_agent.types import ActionCommand


def test_sequence_publisher_waits_for_each_successful_result():
    sequencer = SequentialActionPublisher(result_timeout_s=0.01)
    sent = []

    def publish(command):
        sent.append(command.name)
        sequencer.notify_result(command.request_id, True, "succeeded")

    report = sequencer.publish(
        [
            ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.2}),
            ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        ],
        publish,
        wait_for_results=True,
    )

    assert sent == ["move", "turn"]
    assert report.published == 2
    assert report.completed == 2
    assert not report.failed


def test_sequence_publisher_stops_after_failed_step():
    sequencer = SequentialActionPublisher(result_timeout_s=0.01)
    sent = []

    def publish(command):
        sent.append(command.name)
        if sent[-1] == "move":
            sequencer.notify_result(command.request_id, False, "blocked", status=5)

    report = sequencer.publish(
        [
            ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.2}),
            ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        ],
        publish,
        wait_for_results=True,
    )

    # 整个批次先交给 C++ scheduler；首步失败后 STOP 会清掉尚未执行的 turn。
    assert sent == ["move", "turn", "stop"]
    assert report.failed
    assert report.reason == "blocked"


def test_single_action_can_be_published_without_waiting_for_executor():
    sequencer = SequentialActionPublisher(result_timeout_s=0.01)
    sent = []

    report = sequencer.publish(
        [ActionCommand("move", {"linear_x": 0.2, "duration_s": 1.0})],
        sent.append,
        wait_for_results=False,
    )

    assert len(sent) == 1
    assert report.published == 1
    assert not report.failed


def test_sequence_publisher_waits_for_matching_command_id():
    sequencer = SequentialActionPublisher(result_timeout_s=0.01)
    sent = []

    def publish(command):
        sent.append(command)
        sequencer.notify_result(command.request_id, True, "succeeded", status=1)

    report = sequencer.publish(
        [
            ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.2}),
            ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        ],
        publish,
        wait_for_results=True,
    )

    assert [command.name for command in sent] == ["move", "turn"]
    assert sent[0].request_id != sent[1].request_id
    assert report.completed == 2
    assert not report.failed


def test_sequence_publisher_hands_whole_batch_to_cpp_before_waiting():
    sequencer = SequentialActionPublisher(result_timeout_s=0.1)
    sent = []

    def publish(command):
        sent.append(command)
        if len(sent) == 2:
            # 只有两个候选都发布后才模拟 C++ scheduler 返回结果；旧的逐条发布实现
            # 会在第一条上超时，因此这个测试固定了新的职责边界。
            for item in sent:
                sequencer.notify_result(item.request_id, True, "succeeded", status=1)

    report = sequencer.publish(
        [ActionCommand("move", {}), ActionCommand("turn", {})],
        publish,
        wait_for_results=True,
    )

    assert [item.name for item in sent] == ["move", "turn"]
    assert report.completed == 2
    assert not report.failed


def test_sequence_publisher_can_be_cancelled_while_waiting_for_result():
    sequencer = SequentialActionPublisher(result_timeout_s=1.0)
    sent = []

    def publish(command):
        sent.append(command.name)
        sequencer.cancel("priority_stop")

    report = sequencer.publish(
        [
            ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.2}),
            ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        ],
        publish,
        wait_for_results=True,
    )

    assert sent == ["move"]
    assert report.failed
    assert report.reason == "priority_stop"


def test_sequence_publisher_does_not_continue_when_cancel_races_with_success():
    sequencer = SequentialActionPublisher(result_timeout_s=1.0)
    sent = []

    def publish(command):
        sent.append(command.name)
        sequencer.notify_result(command.request_id, True, "succeeded")
        sequencer.cancel("priority_stop")

    report = sequencer.publish(
        [
            ActionCommand("move", {"linear_x": 0.18, "duration_s": 1.2}),
            ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        ],
        publish,
        wait_for_results=True,
    )

    assert sent == ["move"]
    assert report.failed
    assert report.reason == "priority_stop"
