#!/usr/bin/env python3
"""Configure and activate one lifecycle node without the ros2cli daemon."""

import argparse

import rclpy
from embodied_agent_bringup.lifecycle_state_wait import wait_for_state
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleActivator(Node):
    def __init__(self, target):
        super().__init__("lifecycle_activator")
        prefix = "/" + target.strip("/")
        self.change = self.create_client(ChangeState, prefix + "/change_state")
        self.get_state = self.create_client(GetState, prefix + "/get_state")

    def wait(self, timeout_s=8.0):
        if not self.change.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("change_state service was not discovered")
        if not self.get_state.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError("get_state service was not discovered")

    def state(self):
        future = self.get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is None:
            raise RuntimeError("failed to read lifecycle state")
        return future.result().current_state.id

    def transition(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None or not future.result().success:
            raise RuntimeError(f"transition {transition_id} was rejected")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    parser.add_argument(
        "--target-state",
        choices=("active", "inactive", "unconfigured"),
        default="active",
    )
    parser.add_argument(
        "--wait-only",
        action="store_true",
        help="只观察目标状态，不主动触发生命周期转换",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = LifecycleActivator(args.node_name)
    try:
        node.wait(timeout_s=min(8.0, max(0.1, args.timeout)))
        expected = {
            "active": State.PRIMARY_STATE_ACTIVE,
            "inactive": State.PRIMARY_STATE_INACTIVE,
            "unconfigured": State.PRIMARY_STATE_UNCONFIGURED,
        }[args.target_state]
        if args.wait_only:
            # Lifecycle manager 拥有转换权；验收探针只读取状态，避免双重 configure/activate。
            wait_for_state(
                node.state,
                expected_state=expected,
                timeout_s=args.timeout,
            )
            return
        state = node.state()
        if args.target_state == "unconfigured" and state == State.PRIMARY_STATE_ACTIVE:
            node.transition(Transition.TRANSITION_DEACTIVATE)
            state = node.state()
        if args.target_state == "unconfigured" and state == State.PRIMARY_STATE_INACTIVE:
            node.transition(Transition.TRANSITION_CLEANUP)
            state = node.state()
        if args.target_state in ("inactive", "active") and state == State.PRIMARY_STATE_UNCONFIGURED:
            node.transition(Transition.TRANSITION_CONFIGURE)
            state = node.state()
        if args.target_state == "inactive" and state == State.PRIMARY_STATE_ACTIVE:
            node.transition(Transition.TRANSITION_DEACTIVATE)
            state = node.state()
        if args.target_state == "active" and state == State.PRIMARY_STATE_INACTIVE:
            node.transition(Transition.TRANSITION_ACTIVATE)
            state = node.state()
        if state != expected:
            raise RuntimeError(
                f"node did not reach {args.target_state}; state={state}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
