"""SLAM/Nav2 probe 的阻塞式 Action/Service 操作 Adapter。"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any, Protocol

from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String


__all__ = (
    "SessionActionNode",
    "lifecycle_states",
    "nav_path_points",
    "request_path",
    "run_text_action",
    "wait_until",
)


class SessionActionNode(Protocol):
    """这些 helper 真正依赖的最小观察器接口。"""

    compute_path_client: Any
    lifecycle_clients: dict[str, Any]
    candidates: list[dict]
    text_pub: Any

    def get_clock(self) -> Any: ...

    def result_for(self, command_id: str) -> dict | None: ...


def wait_until(predicate: Callable[[], bool], timeout: float, description: str) -> None:
    """等待由 ROS executor 异步更新的条件，超时给出可操作的错误。"""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def request_path(
    node: SessionActionNode, goal_x: float, goal_y: float
) -> NavPath:
    if not node.compute_path_client.wait_for_server(timeout_sec=20.0):
        raise TimeoutError("ComputePathToPose action server unavailable")
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = "map"
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = goal_x
    goal.goal.pose.position.y = goal_y
    goal.goal.pose.orientation.w = 1.0
    future = node.compute_path_client.send_goal_async(goal)
    wait_until(future.done, 10.0, "ComputePathToPose response timeout")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("ComputePathToPose goal rejected")
    result_future = handle.get_result_async()
    wait_until(result_future.done, 20.0, "ComputePathToPose result timeout")
    wrapped = result_future.result()
    if (
        wrapped.status != GoalStatus.STATUS_SUCCEEDED
        or len(wrapped.result.path.poses) < 5
    ):
        raise RuntimeError(
            f"ComputePathToPose failed status={wrapped.status} "
            f"error={wrapped.result.error_code}: {wrapped.result.error_msg}"
        )
    return wrapped.result.path


def nav_path_points(
    path: NavPath, *, sampled: bool = False
) -> tuple[tuple[float, float], ...]:
    """ROS Path Adapter：纯证据模块只接收二维点，不依赖 nav_msgs。"""

    if not path.poses:
        return ()
    stride = max(1, len(path.poses) // 20) if sampled else 1
    return tuple(
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in path.poses[::stride]
    )


def lifecycle_states(node: SessionActionNode) -> dict[str, int]:
    states: dict[str, int] = {}
    for name, client in node.lifecycle_clients.items():
        if not client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError(f"{name} lifecycle service unavailable")
        future = client.call_async(GetState.Request())
        wait_until(future.done, 5.0, f"{name} lifecycle response timeout")
        states[name] = int(future.result().current_state.id)
    return states


def run_text_action(
    node: SessionActionNode,
    *,
    text: str,
    expected_action: str,
    timeout: float,
) -> dict:
    """通过 Agent 文本入口执行一步；自动门禁只替换声学 ASR，不绕过控制链。"""

    candidate_start = len(node.candidates)
    node.text_pub.publish(String(data=text))
    wait_until(
        lambda: any(
            item.get("name") == expected_action
            for item in node.candidates[candidate_start:]
        ),
        15.0,
        f"no {expected_action} candidate for {text!r}",
    )
    candidate = next(
        item
        for item in node.candidates[candidate_start:]
        if item.get("name") == expected_action
    )
    # 不能用“最近一条 result”推进流程：上一个 Action 的迟到结果可能串台。
    # request_id/command_id 相关后，只有本次候选对应的终态才能解除等待。
    command_id = str(candidate.get("request_id") or "")
    if not command_id:
        raise RuntimeError(f"candidate has no command id: {candidate}")
    wait_until(
        lambda: node.result_for(command_id) is not None,
        timeout,
        f"action result timeout for {text!r}",
    )
    result = node.result_for(command_id)
    if result is None or result.get("success") is not True:
        raise RuntimeError(f"action failed for {text!r}: {result}")
    return {"text": text, "candidate": candidate, "result": result}
