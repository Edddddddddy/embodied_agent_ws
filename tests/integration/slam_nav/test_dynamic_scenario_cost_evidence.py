from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
import pytest

from tools.acceptance.probes.slam_nav import dynamic_cost_evidence
from tools.acceptance.probes.slam_nav import dynamic_scenario
from tools.acceptance.probes.slam_nav.dynamic_scenario import (
    path_relative_motion_positions,
)


class _CostNode:
    def __init__(self, costs: list[int]) -> None:
        self._costs = iter(costs)

    def cost_at(self, _x: float, _y: float) -> int:
        return next(self._costs)


class _FreeCostmapNode:
    def cost_at(self, _x: float, _y: float) -> int:
        return 0


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _PreGoalFailureNode:
    """模拟动态 goal 尚未创建时，上一阶段已经安全停车。"""

    latest_tracks = None

    def __init__(self) -> None:
        self.logger = _Logger()

    def publish_empty_detection(self) -> None:
        return None

    def has_fresh_terminal_stop(self) -> bool:
        return True

    def has_fresh_final_motion_stop(self) -> bool:
        # 准备阶段尚未发动态 goal，本来就不可能产生“运动后停车”证据。
        return False

    def get_logger(self) -> _Logger:
        return self.logger


def test_path_relative_motion_builds_a_replan_anchor_on_a_straight_path() -> None:
    """动态障碍轨迹生成必须在普通测试中真正执行，防止漏导入运行到现场才暴露。"""

    path = NavPath()
    for index in range(31):
        pose = PoseStamped()
        pose.pose.position.x = index * 0.1
        path.poses.append(pose)
    scenario = {
        "prediction_horizon_s": 2.0,
        "path_relative_motion": {
            "path_fraction": 0.5,
            "minimum_anchor_lateral_clearance_m": 0.9,
            "anchor_search_max_clearance_m": 1.5,
            "anchor_lethal_cost": 253,
            "speed_mps": 0.25,
        },
        "warmup": {"sample_count": 3, "interval_s": 0.2},
        "navigation": {"sample_count": 4, "interval_s": 0.2},
    }

    warmup, navigation, anchor = path_relative_motion_positions(
        _FreeCostmapNode(), path, scenario
    )

    assert len(warmup) == 3
    assert len(navigation) == 4
    assert anchor["tangent_x"] == 1.0
    assert anchor["tangent_y"] == 0.0
    assert anchor["lateral_clearance_m"] >= 0.9


def test_route_preparation_failure_cleanup_requires_stop_not_new_motion(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scenario_path = (
        Path(__file__).parents[3]
        / "src/embodied_navigation/config/showcase_dynamic_obstacle_scenario.json"
    )
    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    scenario["route_selection"]["reset_wait_s"] = 0.0
    local_scenario = tmp_path / "scenario.json"
    local_scenario.write_text(json.dumps(scenario), encoding="utf-8")
    node = _PreGoalFailureNode()

    monkeypatch.setattr(
        dynamic_scenario,
        "set_gazebo_entity_pose",
        lambda **_kwargs: None,
    )

    def fail_before_goal(*_args, **_kwargs):
        raise RuntimeError("route preparation failed")

    monkeypatch.setattr(
        dynamic_scenario,
        "select_replannable_route",
        fail_before_goal,
    )
    monkeypatch.setattr(
        dynamic_scenario,
        "wait_until",
        lambda predicate, _timeout, description: (
            None
            if predicate()
            else (_ for _ in ()).throw(TimeoutError(description))
        ),
    )

    with pytest.raises(RuntimeError, match="route preparation failed"):
        dynamic_scenario.run_showcase_dynamic_navigation(
            node,
            local_scenario,
            timeout_s=1.0,
        )

    # 原始错误前没有动态运动；只要已有新零速，清理不得再制造一个伪失败。
    assert node.logger.warnings == []


def _poll_twice(
    predicate: Callable[[], bool],
    _timeout_s: float,
    _description: str,
) -> None:
    assert predicate() is False
    assert predicate() is True


def test_confirmed_lethal_cost_survives_route_commit_decay(monkeypatch) -> None:
    """复现 cost 先到 lethal、规划结束前又按 TTL 衰减的现场竞态。"""

    observed_times = iter((10.0, 10.1))
    monkeypatch.setattr(dynamic_cost_evidence, "wait_until", _poll_twice)
    monkeypatch.setattr(
        dynamic_cost_evidence.time,
        "monotonic",
        lambda: next(observed_times),
    )

    confirmation = dynamic_cost_evidence.confirm_predicted_lethal_cost(
        _CostNode([199, 254]),
        x=1.7,
        y=3.6,
        minimum_cost=253,
        timeout_s=5.0,
    )
    fields = dynamic_cost_evidence.prediction_cost_evidence_fields(
        confirmation,
        route_commit_cost=199,
    )

    # 兼容字段 cost 必须使用已确认的 lethal 快照，不能被稍后的 TTL 清理覆盖。
    assert fields["cost"] == 254
    assert fields["confirmed_cost"] == 254
    assert fields["first_lethal_cost"] == 254
    assert fields["first_lethal_at_monotonic_s"] == 10.1
    assert fields["maximum_observed_cost"] == 254
    assert fields["maximum_observed_at_monotonic_s"] == 10.1
    assert fields["route_commit_cost"] == 199


def test_confirmation_records_first_and_maximum_lethal_samples() -> None:
    latch = dynamic_cost_evidence.PredictedCostLatch(minimum_cost=253)

    assert latch.observe(200, observed_at_monotonic_s=1.0) is False
    assert latch.observe(253, observed_at_monotonic_s=2.0) is True
    assert latch.observe(254, observed_at_monotonic_s=3.0) is True
    confirmation = latch.confirmation()

    assert confirmation.first_lethal_cost == 253
    assert confirmation.first_lethal_at_monotonic_s == 2.0
    assert confirmation.maximum_observed_cost == 254
    assert confirmation.maximum_observed_at_monotonic_s == 3.0
