#!/usr/bin/env python3
"""Checks for the human-facing continuous Nav2 voice launcher."""

import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def _run_showcase(
    command: str,
    mode: str,
    env: dict[str, str],
    *,
    timeout: float | None = None,
) -> subprocess.CompletedProcess:
    """通过公开 shell 入口验证阶段契约。"""

    return subprocess.run(
        [
            "bash",
            str(ROOT / "scripts" / "voice_slam_nav_showcase.sh"),
            command,
            mode,
        ],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=True,
    )


def test_continuous_nav2_voice_control_prints_nav2_launch_config():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CONTROL_PROFILE": "quiet",
            "VOICE_SESSION_TIMEOUT": "166",
            "CONTINUOUS_COMMAND_QUEUE_SIZE": "7",
            "NAV_ACTION_TIMEOUT_S": "321.0",
            "NAV2_INITIAL_X": "-1.8",
            "NAV2_INITIAL_Y": "-0.4",
            "NAV2_INITIAL_YAW": "0.2",
            "SPEECH_END_SILENCE_S": "0.66",
            "ASR_COMMIT_DELAY_MS": "250",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "online"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "Nav2 连续语音导航模式=online" in result.stdout
    assert "去门口" in result.stdout
    assert "依次去门口、书桌、起点" in result.stdout
    assert "VOICE_SESSION_TIMEOUT=166" in result.stdout
    assert "CONTINUOUS_COMMAND_QUEUE_SIZE=7" in result.stdout
    assert "NAV_ACTION_TIMEOUT_S=321.0" in result.stdout
    assert "NAV2_INITIAL_X=-1.8" in result.stdout
    assert "NAV2_INITIAL_Y=-0.4" in result.stdout
    assert "NAV2_INITIAL_YAW=0.2" in result.stdout
    assert "VAD_PROVIDER=energy" in result.stdout
    assert "voice_nav2_turtlebot3.launch.py" in result.stdout
    assert "INITIAL_POSE_POLICY=amcl" in result.stdout
    assert "microphone_enabled:=true" in result.stdout
    assert "continuous_control_enabled:=true" in result.stdout
    assert "nav_action_timeout_s:=321.0" in result.stdout
    assert "x_pose:=-1.8" in result.stdout
    assert "y_pose:=-0.4" in result.stdout
    assert "yaw:=0.2" in result.stdout
    assert "speech_end_silence_s:=0.66" in result.stdout
    assert "asr_commit_delay_ms:=250" in result.stdout
    assert "control_authority_enabled:=false" in result.stdout
    assert "control_authority_manager_enabled:=false" in result.stdout


def test_persistent_voice_base_prints_only_the_session_base_launch():
    """持久 base 只拥有一次性资源，不得重新夹带 mapping/navigation provider。"""

    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "SHOWCASE_PERSISTENT_SESSION": "true",
            "SHOWCASE_SESSION_DIR": "/tmp/showcase-session-a",
            "SHOWCASE_MAP_PREFIX": "/tmp/showcase-session-a/voice_built_map",
            "CONTROL_AUTHORITY_ENABLED": "true",
            "NAV2_WORLD": "/tmp/showcase-world.sdf.xacro",
            "NAV2_PARAMS_FILE": "/tmp/showcase-nav2.yaml",
            "ROS_DOMAIN_ID": "171",
            "GZ_PARTITION": "showcase_partition_171",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "SHOWCASE_PERSISTENT_SESSION=true" in result.stdout
    assert "SHOWCASE_SESSION_DIR=/tmp/showcase-session-a" in result.stdout
    assert "SHOWCASE_MAP_PREFIX=/tmp/showcase-session-a/voice_built_map" in result.stdout
    assert "SYSTEM_READINESS_PROFILE=persistent_mapping_stage" in result.stdout
    assert "INITIAL_POSE_POLICY=stage" in result.stdout
    assert "ROS_DOMAIN_ID=171" in result.stdout
    assert "GZ_PARTITION=showcase_partition_171" in result.stdout
    assert "persistent_voice_nav_base.launch.py" in result.stdout
    assert "control_authority_enabled:=true" in result.stdout
    assert "world:=/tmp/showcase-world.sdf.xacro" in result.stdout
    assert "params_file:=/tmp/showcase-nav2.yaml" in result.stdout
    assert "voice_nav2_turtlebot3.launch.py" not in result.stdout
    assert "slam:=" not in result.stdout
    assert "map:=" not in result.stdout
    assert "executor_plugin:=" not in result.stdout
    assert "control_authority_manager_enabled:=" not in result.stdout
    assert "nav_action_timeout_s:=" not in result.stdout
    assert "authority_state_heartbeat_ms:=" not in result.stdout


def test_showcase_base_delegates_to_one_persistent_voice_runtime():
    """StageProcessManager 的 base 公共命令应复用 continuous 语音支持层。"""

    env = os.environ.copy()
    env.update(
        {
            "SHOWCASE_PRINT_CONFIG": "true",
            "SHOWCASE_SESSION_DIR": "/tmp/showcase-session-b",
            "SHOWCASE_MAP_PREFIX": "/tmp/showcase-session-b/fresh_map",
            "ROS_DOMAIN_ID": "172",
            "GZ_PARTITION": "showcase_partition_172",
            "VAD_PROVIDER": "energy",
        }
    )

    result = _run_showcase("base", "online", env)

    assert "[showcase] 持久 base" in result.stdout
    assert "Nav2 连续语音导航模式=online" in result.stdout
    assert "SHOWCASE_PERSISTENT_SESSION=true" in result.stdout
    assert "SHOWCASE_SESSION_DIR=/tmp/showcase-session-b" in result.stdout
    assert "SHOWCASE_MAP_PREFIX=/tmp/showcase-session-b/fresh_map" in result.stdout
    assert "ROS_DOMAIN_ID=172" in result.stdout
    assert "GZ_PARTITION=showcase_partition_172" in result.stdout
    assert result.stdout.count("persistent_voice_nav_base.launch.py") == 1
    assert (
        "params_file:=/tmp/showcase-session-b/showcase_dynamic_nav2_params.yaml"
        in result.stdout
    )


def test_persistent_mapping_starts_only_the_replaceable_mapping_stage(tmp_path):
    session_dir = tmp_path / "persistent-session"
    env = os.environ.copy()
    env.update(
        {
            "SHOWCASE_PRINT_CONFIG": "true",
            "SHOWCASE_PERSISTENT_SESSION": "true",
            "SHOWCASE_SESSION_DIR": str(session_dir),
            "SHOWCASE_MAP_PREFIX": str(session_dir / "fresh_map"),
            "ROS_DOMAIN_ID": "173",
            "GZ_PARTITION": "showcase_partition_173",
        }
    )

    result = _run_showcase("mapping", "offline", env)

    assert "ROS_DOMAIN_ID=173" in result.stdout
    assert "GZ_PARTITION=showcase_partition_173" in result.stdout
    assert "persistent_mapping_stage.launch.py" in result.stdout
    assert "readiness_stale_timeout_s:=30.0" in result.stdout
    assert (
        f"params_file:={session_dir / 'showcase_dynamic_nav2_params.yaml'}"
        in result.stdout
    )
    assert "persistent_voice_nav_base.launch.py" not in result.stdout
    assert "continuous_nav2_voice_control.sh" not in result.stdout
    assert "voice_nav2_turtlebot3.launch.py" not in result.stdout
    assert "persistent_navigation_stage.launch.py" not in result.stdout


def test_persistent_navigation_reuses_fresh_map_and_session_nav2_params(tmp_path):
    session_dir = tmp_path / "persistent-session"
    session_dir.mkdir()
    map_prefix = session_dir / "fresh_map"
    map_prefix.with_suffix(".yaml").write_text(
        "image: fresh_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update(
        {
            "SHOWCASE_PRINT_CONFIG": "true",
            "SHOWCASE_PERSISTENT_SESSION": "true",
            "SHOWCASE_SESSION_DIR": str(session_dir),
            "SHOWCASE_MAP_PREFIX": str(map_prefix),
            "ROS_DOMAIN_ID": "174",
            "GZ_PARTITION": "showcase_partition_174",
        }
    )

    result = _run_showcase("navigation", "offline", env)

    assert "ROS_DOMAIN_ID=174" in result.stdout
    assert "GZ_PARTITION=showcase_partition_174" in result.stdout
    assert "persistent_navigation_stage.launch.py" in result.stdout
    assert "readiness_stale_timeout_s:=30.0" in result.stdout
    assert "enable_dynamic_obstacle_layer:=true" in result.stdout
    assert f"map:={map_prefix.with_suffix('.yaml')}" in result.stdout
    assert f"params_file:={session_dir / 'showcase_dynamic_nav2_params.yaml'}" in result.stdout
    assert "persistent_voice_nav_base.launch.py" not in result.stdout
    assert "continuous_nav2_voice_control.sh" not in result.stdout
    assert "voice_nav2_turtlebot3.launch.py" not in result.stdout
    assert "persistent_mapping_stage.launch.py" not in result.stdout


def test_showcase_auto_explicitly_opts_orchestrator_into_persistent_runtime(tmp_path):
    session_dir = tmp_path / "persistent-session"
    env = os.environ.copy()
    env.update(
        {
            "SHOWCASE_PRINT_CONFIG": "true",
            "SHOWCASE_PERSISTENT_SESSION": "true",
            "SHOWCASE_SESSION_DIR": str(session_dir),
            "SHOWCASE_MAP_PREFIX": str(session_dir / "fresh_map"),
            "ROS_DOMAIN_ID": "175",
            "GZ_PARTITION": "showcase_partition_175",
        }
    )

    result = _run_showcase("auto", "offline", env)

    assert "voice_slam_session_orchestrator" in result.stdout
    assert "persistent_runtime_enabled:=true" in result.stdout
    # 持久切换必须经过 HOLD/ACK/RESUME，不能依赖用户再记一个隐藏开关。
    assert "authority_gate_enabled:=true" in result.stdout
    assert f"map_prefix:={session_dir / 'fresh_map'}" in result.stdout
    assert "ROS_DOMAIN_ID=175" in result.stdout
    assert "GZ_PARTITION=showcase_partition_175" in result.stdout
    assert "control_authority" not in result.stdout


def test_showcase_auto_keeps_strict_legacy_runtime_by_default(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "SHOWCASE_PRINT_CONFIG": "true",
            "SHOWCASE_SESSION_DIR": str(tmp_path / "strict-session"),
            "ROS_DOMAIN_ID": "177",
            "GZ_PARTITION": "showcase_partition_177",
        }
    )
    env.pop("SHOWCASE_PERSISTENT_SESSION", None)

    result = _run_showcase("auto", "offline", env)

    assert "persistent_runtime_enabled:=false" in result.stdout
    assert "persistent_voice_nav_base.launch.py" not in result.stdout


def test_showcase_mapping_keeps_legacy_full_stage_without_persistent_opt_in(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "SHOWCASE_PRINT_CONFIG": "true",
            "SHOWCASE_PERSISTENT_SESSION": "false",
            "SHOWCASE_SESSION_DIR": str(tmp_path / "legacy-session"),
            "ROS_DOMAIN_ID": "176",
            "GZ_PARTITION": "showcase_partition_176",
            "VAD_PROVIDER": "energy",
        }
    )

    result = _run_showcase("mapping", "offline", env, timeout=5.0)

    assert "SHOWCASE_PERSISTENT_SESSION=false" in result.stdout
    assert "SYSTEM_READINESS_PROFILE=voice_nav2" in result.stdout
    assert "INITIAL_POSE_POLICY=slam" in result.stdout
    assert "voice_nav2_turtlebot3.launch.py" in result.stdout
    assert "slam:=true" in result.stdout
    assert "executor_plugin:=embodied_simulation/GazeboRobotExecutor" in result.stdout
    assert "persistent_voice_nav_base.launch.py" not in result.stdout
    assert "persistent_mapping_stage.launch.py" not in result.stdout


def test_stage_rejects_a_self_contained_manager_without_coordinator():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "CONTROL_AUTHORITY_ENABLED": "true",
            "CONTROL_AUTHORITY_MANAGER_ENABLED": "true",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 2
    assert "cannot own control_authority" in result.stderr
    assert "voice_slam_nav_showcase.sh auto" in result.stderr


def test_authority_enabled_stage_declares_an_external_session_manager():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "CONTROL_AUTHORITY_ENABLED": "true",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "CONTROL_AUTHORITY_ENABLED=true" in result.stdout
    assert "CONTROL_AUTHORITY_MANAGER_ENABLED=false" in result.stdout
    assert "control_authority_enabled:=true" in result.stdout
    assert "control_authority_manager_enabled:=false" in result.stdout


def test_continuous_nav2_voice_control_low_gain_profile_lowers_vad_and_disables_aec():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "VOICE_CONTROL_PROFILE": "low_gain",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0012" in result.stdout
    assert "SPEECH_END_SILENCE_S=0.85" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=450" in result.stdout
    assert "AEC_ENABLED=false" in result.stdout
    assert "speech_start_threshold:=0.0012" in result.stdout
    assert "speech_end_silence_s:=0.85" in result.stdout
    assert "asr_commit_delay_ms:=450" in result.stdout
    assert "aec_enabled:=false" in result.stdout


def test_nav2_voice_control_reuses_calibration_and_wsl_pulse_bridge(tmp_path):
    calibration = tmp_path / "voice_calibration.env"
    calibration.write_text(
        "\n".join(
            [
                "export VOICE_CONTROL_PROFILE=low_gain",
                "export SPEECH_START_THRESHOLD=0.0016",
                "export ASR_COMMIT_DELAY_MS=525",
                "export VAD_PROVIDER=energy",
                "export KWS_PROVIDER=openwakeword",
                "export AEC_ENABLED=false",
            ]
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    for key in (
        "VOICE_CONTROL_PROFILE",
        "SPEECH_START_THRESHOLD",
        "ASR_COMMIT_DELAY_MS",
        "VAD_PROVIDER",
        "AEC_ENABLED",
    ):
        env.pop(key, None)
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "APPLY_VOICE_CALIBRATION": "true",
            "VOICE_CALIBRATION_ENV": str(calibration),
            "PULSE_CAPTURE_BRIDGE": "auto",
            "PULSE_SERVER": "unix:/mnt/wslg/PulseServer",
            "KWS_PROVIDER": "sherpa",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "VOICE_CONTROL_PROFILE=low_gain" in result.stdout
    assert "VOICE_CALIBRATION_ENV=" in result.stdout
    assert "applied=true" in result.stdout
    assert "SPEECH_START_THRESHOLD=0.0016" in result.stdout
    assert "ASR_COMMIT_DELAY_MS=525" in result.stdout
    assert "KWS_PROVIDER=sherpa" in result.stdout
    assert "PULSE_CAPTURE_BRIDGE=auto（active=true" in result.stdout
    assert "capture_enabled:=false" in result.stdout


def test_nav2_voice_control_disables_pulse_bridge_without_microphone():
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(ROOT),
            "CONTINUOUS_PRINT_CONFIG": "true",
            "NAV2_MICROPHONE_ENABLED": "false",
            "NAV2_CAPTURE_ENABLED": "false",
            "PULSE_CAPTURE_BRIDGE": "auto",
            "PULSE_SERVER": "unix:/mnt/wslg/PulseServer",
            "VAD_PROVIDER": "energy",
        }
    )

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "offline"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )

    assert "MICROPHONE_ENABLED=false" in result.stdout
    assert "PULSE_CAPTURE_BRIDGE=auto（active=false" in result.stdout


def test_nav2_voice_control_health_checks_pulse_before_launch():
    script = (ROOT / "scripts" / "continuous_nav2_voice_control.sh").read_text(
        encoding="utf-8"
    )

    bridge_call = script.index("start_pulse_capture_bridge\nbuild_launch_args")
    launch_call = script.index('setsid ros2 launch "${LAUNCH_ARGS[@]}"')
    assert bridge_call < launch_call
    assert 'kill -0 "$PULSE_BRIDGE_PID"' in script
    assert "PulseAudio capture bridge startup failed" in script


def test_continuous_nav2_voice_control_rejects_unknown_mode():
    env = os.environ.copy()
    env.update({"WORKSPACE": str(ROOT), "CONTINUOUS_PRINT_CONFIG": "true"})

    result = subprocess.run(
        ["bash", str(ROOT / "scripts" / "continuous_nav2_voice_control.sh"), "cloud"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 2
    assert "Usage:" in result.stderr
