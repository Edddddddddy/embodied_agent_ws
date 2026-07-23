#!/usr/bin/env python3
"""编排语音意图驱动的 SLAM 建图、定位与 Nav2 导航验收会话。"""

from __future__ import annotations

import json
import re
import threading
import time
from pathlib import Path

from embodied_agent_interfaces.action import ExecuteRobotCommand, ManageSlamSession
from embodied_agent_interfaces.msg import (
    RobotCommand,
    SlamSessionState,
)
from lifecycle_msgs.msg import State
import rclpy
from std_msgs.msg import String
from tools.acceptance.probes.slam_nav.artifacts import (
    build_failure_report,
    load_survey_plan,
    map_artifact_sha256,
    print_report,
    robot_traveled_distance,
    sha256_file,
)
from tools.acceptance.probes.slam_nav.cli import build_parser
from tools.acceptance.probes.slam_nav.dynamic_scenario import (
    run_showcase_dynamic_navigation,
)
from tools.acceptance.probes.slam_nav.live_voice_trigger import (
    await_automatic_mission_trigger,
    finalize_trigger_report,
)
from tools.acceptance.probes.slam_nav.session_observer import (
    GazeboTruthConfig,
    SessionObserver,
    lifecycle_states,
    run_text_action,
    wait_until,
)
from tools.acceptance.probes.slam_nav.session_runtime import (
    run_cli,
    spin_executor_until_shutdown,
    wait_for_mapping_startup,
)
from tools.acceptance.probes.slam_nav.unknown_world_report_adapter import (
    build_session_report as build_unknown_world_session_report,
)
from tools.acceptance.progress import AcceptanceProgress
from tools.acceptance.slam_nav_evidence import (
    AutomaticMissionObservation,
    AutomaticMissionThresholds,
    build_automatic_mission_report,
)
from tools.acceptance.unknown_world_evidence import load_scene_evaluation_context
import yaml


def _publish_state_progress(
    progress: AcceptanceProgress, message: SlamSessionState
) -> None:
    """把细粒度状态机压缩成操作者关心的六个验收里程碑。"""

    milestone = {
        SlamSessionState.STARTING_MAPPING: (1, "runtime_startup"),
        SlamSessionState.MAPPING: (1, "runtime_startup"),
        SlamSessionState.AUTOMATIC_MAPPING: (2, "frontier_slam"),
        SlamSessionState.SAVING_MAP: (3, "map_save"),
        SlamSessionState.MAP_SAVED: (3, "map_save"),
        SlamSessionState.SWITCHING_TO_NAVIGATION: (
            4,
            "localization_and_semantic_nav",
        ),
        SlamSessionState.STARTING_NAVIGATION: (
            4,
            "localization_and_semantic_nav",
        ),
        SlamSessionState.NAVIGATING: (4, "localization_and_semantic_nav"),
        SlamSessionState.AUTOMATIC_NAVIGATING: (
            4,
            "localization_and_semantic_nav",
        ),
    }.get(int(message.phase))
    if milestone is not None:
        # AcceptanceProgress 自带去重，transient-local 重投递不会刷屏。
        progress.stage(*milestone, detail=str(message.detail))


def main() -> None:
    args = build_parser().parse_args()
    live_voice_trigger = args.automatic_trigger_source == "live_voice"
    if live_voice_trigger and not args.automatic_mission:
        raise ValueError("live_voice trigger requires --automatic-mission")
    if live_voice_trigger and args.cancel_automatic_mission:
        raise ValueError("live_voice trigger does not support synthetic cancel")
    scene_context = None
    gazebo_truth_config = None
    if args.unknown_world:
        if args.scene_spec is None or args.truth_map is None:
            raise ValueError(
                "--unknown-world requires evaluator-only --scene-spec and --truth-map"
            )
        scene_context = load_scene_evaluation_context(args.scene_spec)
        gazebo_truth_config = GazeboTruthConfig(
            world_name=scene_context.world_name,
            robot_entity_name=args.gazebo_robot_entity,
            transform=scene_context.transform,
        )
    progress = (
        AcceptanceProgress(
            label=(
                "unknown-world-slam-e2e"
                if args.unknown_world
                else "slam-nav-e2e"
            ),
            total_stages=6,
            heartbeat_s=args.progress_heartbeat_s,
        )
        if args.automatic_mission
        else None
    )
    progress_outcome = "FAIL"
    rclpy.init()
    state_observer = (
        (lambda message: _publish_state_progress(progress, message))
        if progress is not None
        else None
    )
    node = SessionObserver(
        state_observer=state_observer,
        gazebo_truth_config=gazebo_truth_config,
        synthetic_asr_enabled=not live_voice_trigger,
    )
    if progress is not None:
        progress.start(
            session_id=args.session_id or "unassigned",
            timeout_s=args.gate_timeout_s,
            log_path=str(args.runtime_log or args.output),
            detail_supplier=lambda: (
                f"phase={node.current_phase} "
                f"detail={node.current_detail or '-'}"
            ),
        )
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(
        target=spin_executor_until_shutdown,
        args=(executor,),
        daemon=True,
    )
    thread.start()
    try:
        wait_for_mapping_startup(node, args.transition_timeout)
        if args.automatic_mission or args.cancel_automatic_mission:
            state_start = len(node.states)
            await_automatic_mission_trigger(
                node,
                source=args.automatic_trigger_source,
                timeout_s=args.voice_trigger_timeout,
                agent_mode=args.agent_mode,
            )
            if args.cancel_automatic_mission:
                wait_until(
                    lambda: any(
                        state.phase == SlamSessionState.AUTOMATIC_MAPPING
                        for state in node.states[state_start:]
                    ),
                    args.transition_timeout,
                    "automatic mission did not enter AUTOMATIC_MAPPING",
                )
                assert node.asr_pub is not None
                node.asr_pub.publish(String(data="急停"))
                wait_until(
                    lambda: any(
                        state.phase == SlamSessionState.MAPPING
                        and "canceled" in state.detail
                        for state in node.states[state_start:]
                    ),
                    args.transition_timeout,
                    "urgent stop did not recover automatic mission to MAPPING",
                )
                assert not node.has_phase(SlamSessionState.MISSION_COMPLETED)
                report = {
                    "passed": True,
                    "state_sequence": [int(state.phase) for state in node.states],
                    "automatic_mission_canceled": True,
                    "final_phase": SlamSessionState.MAPPING,
                    "final_cmd_vel": node.last_cmd_vel,
                }
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(
                    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                print(json.dumps(report, ensure_ascii=False, indent=2))
                return
            mission_failures = {
                SlamSessionState.MISSION_FAILED,
                SlamSessionState.MISSION_CANCELED,
            }
            wait_until(
                lambda: (
                    node.has_phase(SlamSessionState.MISSION_COMPLETED)
                    or any(
                        state.phase == SlamSessionState.FAILED
                        or state.mission_outcome in mission_failures
                        for state in node.states[state_start:]
                    )
                ),
                args.transition_timeout,
                "automatic mission did not reach a terminal state",
            )
            failed_state = next(
                (
                    state
                    for state in reversed(node.states[state_start:])
                    if state.phase == SlamSessionState.FAILED
                    or state.mission_outcome in mission_failures
                ),
                None,
            )
            if failed_state is not None:
                # 重型进程已经给出确定失败时立即结束，避免继续等待完整超时，
                # 同时让 smoke 脚本及时打印 orchestrator/Nav2 原始日志。
                raise RuntimeError(
                    failed_state.mission_message or failed_state.detail
                )
            observed = [int(state.phase) for state in node.states]
            # 状态 topic 使用 depth=1 + transient-local：dry-run 阶段切换仅数毫秒，
            # 订阅者可能合理地只收到最新快照。最终状态同时携带 map_saved，故可作为
            # 一条语音已经跨越探索、保存和导航事务边界的稳定验收契约。
            final_state = next(
                state
                for state in reversed(node.states)
                if state.phase == SlamSessionState.MISSION_COMPLETED
            )
            lifecycle = {}
            dynamic_navigation = None
            if args.evidence_kind != "dry_run_process_adapter":
                wait_until(
                    node.has_fresh_terminal_stop,
                    5.0,
                    "automatic mission finished without a fresh final zero velocity",
                )
                wait_until(
                    lambda: node.localization_tf_count > 0 and node.amcl_pose_count > 0,
                    20.0,
                    "saved-map localization evidence missing",
                )
                if args.unknown_world:
                    wait_until(
                        lambda: len(node.localization_evidence()[1]) > 0,
                        20.0,
                        "Gazebo SceneBroadcaster robot truth is missing",
                    )
                lifecycle = lifecycle_states(node)
                if args.dynamic_scenario is not None:
                    if progress is not None:
                        progress.stage(
                            5,
                            "dynamic_obstacle_replan",
                            "semantic patrol complete; injecting moving obstacle",
                        )
                    dynamic_navigation = run_showcase_dynamic_navigation(
                        node,
                        args.dynamic_scenario,
                        timeout_s=args.dynamic_navigation_timeout,
                    )
                    wait_until(
                        node.has_fresh_final_motion_stop,
                        8.0,
                        "dynamic navigation ended without fresh motion/stop evidence",
                    )
                if args.unknown_world:
                    wait_until(
                        lambda: (
                            len(node.localization_evidence()[0]) >= 20
                            and len(node.localization_evidence()[1]) >= 20
                        ),
                        20.0,
                        "insufficient time-aligned AMCL/Gazebo samples",
                    )

            if progress is not None:
                progress.stage(
                    6,
                    "evidence_validation",
                    "validating fresh map, localization, plans and final stop",
                )

            map_yaml = Path(final_state.map_yaml_path) if final_state.map_yaml_path else None
            map_provenance = None
            mission_thresholds = AutomaticMissionThresholds(0.0, 0, 0)
            if map_yaml is not None and map_yaml.is_file():
                artifact_hash, image_path = map_artifact_sha256(map_yaml)
                map_provenance = {
                    "yaml_path": str(map_yaml.resolve()),
                    "image_path": str(image_path.resolve()),
                    "yaml_sha256": sha256_file(map_yaml),
                    "image_sha256": sha256_file(image_path),
                    "artifact_sha256": artifact_hash,
                    "yaml_mtime_ns": map_yaml.stat().st_mtime_ns,
                    "image_mtime_ns": image_path.stat().st_mtime_ns,
                }
            if args.mission_plan is not None and not args.unknown_world:
                mission = yaml.safe_load(
                    args.mission_plan.read_text(encoding="utf-8")
                )
                mission_thresholds = AutomaticMissionThresholds.from_mapping(
                    mission.get("acceptance", {})
                )

            mapping_distance_m = robot_traveled_distance(node.mapping_positions)
            completion_detail = next(
                (
                    state.detail
                    for state in reversed(node.states)
                    if "reason=" in state.detail
                ),
                "",
            )
            reason_match = re.search(r"reason=([^ ]+)", completion_detail)
            completion_reason = reason_match.group(1) if reason_match else ""
            lifecycle_active = len(lifecycle) == 4 and all(
                state == int(State.PRIMARY_STATE_ACTIVE)
                for state in lifecycle.values()
            )
            if args.unknown_world:
                if map_yaml is None or not map_yaml.is_file():
                    raise RuntimeError("unknown-world mission did not save a map")
                if scene_context is None or args.truth_map is None:
                    raise RuntimeError("unknown-world evaluator context is missing")
                core_report = build_unknown_world_session_report(
                    node=node,
                    final_state=final_state,
                    session_id=args.session_id,
                    session_start_ns=args.session_start_ns,
                    built_map_yaml=map_yaml,
                    truth_map_yaml=args.truth_map,
                    scene_context=scene_context,
                    map_provenance=map_provenance,
                    nav2_lifecycle_active=lifecycle_active,
                    dynamic_navigation=dynamic_navigation,
                    mapping_path_m=mapping_distance_m,
                )
                report = finalize_trigger_report(
                    node,
                    core_report,
                    live_voice_trigger=live_voice_trigger,
                    agent_mode=args.agent_mode,
                )
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(
                    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                print_report(report, summary_only=args.summary_only)
                if not report["passed"]:
                    raise SystemExit(1)
                progress_outcome = "PASS"
                return
            observation = AutomaticMissionObservation(
                session_id=args.session_id,
                session_start_ns=args.session_start_ns,
                state_sequence=tuple(observed),
                map_saved=bool(final_state.map_saved),
                map_yaml_path=final_state.map_yaml_path,
                final_phase=int(final_state.phase),
                evidence_kind=args.evidence_kind,
                action_candidates=tuple(node.candidates),
                action_results=tuple(node.results),
                map_stats=node.map_stats,
                map_provenance=map_provenance,
                mapping_path_m=mapping_distance_m,
                frontier_goal_count=len(node.frontier_goal_ids),
                exploration_completion_reason=completion_reason,
                localization_tf_count=node.localization_tf_count,
                amcl_pose_count=node.amcl_pose_count,
                lifecycle_states=lifecycle,
                nav2_lifecycle_active=lifecycle_active,
                dynamic_navigation=dynamic_navigation,
                dynamic_navigation_required=args.dynamic_scenario is not None,
                provenance={
                    "world_sha256": sha256_file(args.world_file)
                    if args.world_file
                    else "",
                    "mission_sha256": sha256_file(args.mission_plan)
                    if args.mission_plan
                    else "",
                    "dynamic_scenario_sha256": sha256_file(args.dynamic_scenario)
                    if args.dynamic_scenario
                    else "",
                },
                final_linear_x=float(node.last_cmd_vel["linear_x"]),
                final_angular_z=float(node.last_cmd_vel["angular_z"]),
            )
            report = build_automatic_mission_report(
                observation, mission_thresholds
            )
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print_report(report, summary_only=args.summary_only)
            if not report["passed"]:
                raise SystemExit(1)
            progress_outcome = "PASS"
            return

        # 非 automatic 的旧验收仍会发送“保存地图/开始导航”，因此保留原先
        # 的 subscriber readiness；live_voice 已在入口处限定为 automatic。
        assert node.asr_pub is not None
        wait_until(
            lambda: node.asr_pub.get_subscription_count() > 0,
            5.0,
            "ASR final subscriber unavailable",
        )
        exploration_succeeded = None
        survey_steps: list[dict] = []
        survey_distance_m = 0.0
        survey_map: dict | None = None
        if args.survey_plan:
            route, thresholds = load_survey_plan(args.survey_plan)
            wait_until(
                lambda: (
                    node.text_pub.get_subscription_count() > 0
                    and node.positions
                    and node.map_stats is not None
                    and node.scan_count >= 3
                ),
                30.0,
                "Agent/odom/map/scan inputs unavailable for mapping survey",
            )
            odom_start = len(node.positions)
            for index, step in enumerate(route, start=1):
                print(
                    f"[mapping {index}/{len(route)}] {step['label']}: {step['text']}",
                    flush=True,
                )
                survey_steps.append(
                    run_text_action(
                        node,
                        text=str(step["text"]),
                        expected_action=str(step["action"]),
                        timeout=25.0,
                    )
                )
            wait_until(
                lambda: node.map_stats is not None
                and node.map_stats["known_cells"]
                >= int(thresholds["min_known_map_cells"]),
                15.0,
                "SLAM map did not reach the required known-cell coverage",
            )
            survey_distance_m = robot_traveled_distance(
                node.positions[odom_start:]
            )
            survey_map = dict(node.map_stats or {})
            assert survey_distance_m >= float(thresholds["min_mapping_path_m"]), (
                survey_distance_m,
                thresholds,
            )
            assert survey_map["occupied_cells"] >= int(
                thresholds["min_occupied_map_cells"]
            )
            exploration_succeeded = True
        elif args.explore_before_save:
            assert node.robot_client.wait_for_server(timeout_sec=10.0)
            explore_goal = ExecuteRobotCommand.Goal()
            explore_goal.command.action_type = RobotCommand.MOVE
            explore_goal.command.linear_x = 0.18
            explore_goal.command.duration_s = 2.0
            explore_goal.command.command_id = "showcase-session-exploration"
            explore_goal.command.source = "slam_session_acceptance"
            explore_future = node.robot_client.send_goal_async(explore_goal)
            wait_until(explore_future.done, 10.0, "exploration goal response missing")
            explore_handle = explore_future.result()
            assert explore_handle.accepted
            explore_result_future = explore_handle.get_result_async()
            wait_until(
                explore_result_future.done,
                30.0,
                "mapping exploration action did not finish",
            )
            exploration_succeeded = bool(
                explore_result_future.result().result.success
            )
            assert exploration_succeeded

        # 语音系统意图负责保存地图；dry-run Adapter 不写伪造文件，但状态契约相同。
        node.asr_pub.publish(String(data="保存地图"))
        wait_until(
            lambda: node.has_phase(SlamSessionState.MAP_SAVED),
            args.transition_timeout,
            "voice save-map command did not reach MAP_SAVED",
        )

        assert node.client.wait_for_server(timeout_sec=5.0)
        goal = ManageSlamSession.Goal()
        goal.command = ManageSlamSession.Goal.START_NAVIGATION

        def on_feedback(message) -> None:
            node.feedback_phases.append(message.feedback.state.phase)

        goal_future = node.client.send_goal_async(goal, feedback_callback=on_feedback)
        wait_until(goal_future.done, 5.0, "session Action goal response missing")
        goal_handle = goal_future.result()
        assert goal_handle.accepted
        result_future = goal_handle.get_result_async()
        wait_until(
            result_future.done,
            args.transition_timeout,
            "session Action result missing",
        )
        result = result_future.result().result
        assert result.success, result.message
        assert result.state.phase == SlamSessionState.NAVIGATING
        assert result.state.map_saved
        observed_phases = {
            *(state.phase for state in node.states),
            *node.feedback_phases,
        }
        assert SlamSessionState.SWITCHING_TO_NAVIGATION in observed_phases
        assert SlamSessionState.STARTING_NAVIGATION in observed_phases

        # ASR 抖动造成的重复“开始导航”是幂等操作，不能把会话打入 FAILED。
        node.asr_pub.publish(String(data="开始导航"))
        node.asr_pub.publish(String(data="开始导航"))
        time.sleep(0.3)
        assert not node.has_phase(SlamSessionState.FAILED)

        report = {
            "passed": True,
            "state_sequence": [int(state.phase) for state in node.states],
            "feedback_phases": node.feedback_phases,
            "map_saved": bool(result.state.map_saved),
            "map_yaml_path": result.state.map_yaml_path,
            "final_phase": int(result.state.phase),
            "evidence_kind": args.evidence_kind,
            "exploration_succeeded": exploration_succeeded,
            "survey_step_count": len(survey_steps),
            "survey_distance_m": round(survey_distance_m, 3),
            "survey_map": survey_map,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except Exception as error:
        # 重型门禁即使在中途失败也必须留下机器可读原因，避免现场只看到超时或卡住。
        last_state = node.states[-1] if node.states else None
        core_failure_report = build_failure_report(
            session_id=args.session_id,
            session_start_ns=args.session_start_ns,
            evidence_kind=(
                "unknown_world_slam_nav_dynamic_replan"
                if args.unknown_world
                else args.evidence_kind
            ),
            error=str(error),
            state_sequence=[int(state.phase) for state in node.states],
            last_state_detail=node.current_detail,
            map_stats=node.map_stats,
            frontier_goal_count=len(node.frontier_goal_ids),
            mapping_path_m=robot_traveled_distance(node.mapping_positions),
            final_cmd_vel=node.last_cmd_vel,
            schema_version=4 if args.unknown_world else 3,
            mission_outcome=(
                int(last_state.mission_outcome) if last_state is not None else 0
            ),
            mission_message=(
                str(last_state.mission_message) if last_state is not None else ""
            ),
            # 异常发生时 Observer 已持有最近一帧强类型 frontier/Action 账本；
            # 直接固化到 failure artifact，避免现场只能解析 runtime.log。
            frontier_telemetry=node.frontier_evidence,
            mapping_completion_evidence=node.mapping_completion_evidence,
        )
        failure_report = finalize_trigger_report(
            node,
            core_failure_report,
            live_voice_trigger=live_voice_trigger,
            agent_mode=args.agent_mode,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(failure_report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print_report(failure_report, summary_only=args.summary_only)
        raise
    finally:
        if progress is not None:
            progress.stop(progress_outcome)
        # 先停止回调生产者再销毁 Node/context；否则 executor 线程可能在节点析构后
        # 继续访问订阅回调，现场表现为退出挂住或偶发 rclpy InvalidHandle。
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    run_cli(main)
