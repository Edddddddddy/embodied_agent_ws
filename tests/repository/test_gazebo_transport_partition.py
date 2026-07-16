from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_gazebo_launches_isolate_transport_by_ros_domain():
    launches = [
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_turtlebot3.launch.py",
        ROOT
        / "src"
        / "embodied_simulation"
        / "launch"
        / "voice_nav2_turtlebot3.launch.py",
    ]

    for path in launches:
        content = path.read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument("gz_partition"' in content, path
        assert 'SetEnvironmentVariable("GZ_PARTITION", gz_partition)' in content, path
        assert 'SetEnvironmentVariable("IGN_PARTITION", gz_partition)' in content, path
        assert 'os.environ.get("ROS_DOMAIN_ID", "0")' in content, path


def test_continuous_entrypoints_export_and_print_the_partition():
    entrypoints = [
        ROOT / "scripts" / "continuous_voice_control.sh",
        ROOT / "scripts" / "continuous_nav2_voice_control.sh",
    ]

    for path in entrypoints:
        content = path.read_text(encoding="utf-8")
        assert 'export GZ_PARTITION="${GZ_PARTITION:-embodied_agent_${ROS_DOMAIN_ID}}"' in content
        assert 'GZ_PARTITION=$GZ_PARTITION' in content
