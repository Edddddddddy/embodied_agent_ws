from setuptools import find_packages, setup


package_name = "embodied_slam_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Edddddddddy",
    maintainer_email="Edddddddddy@users.noreply.github.com",
    description="SLAM replay, trajectory evidence, and voice showcase session adapters",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "openloris_rosbag_inspect = embodied_slam_tools.bag_source:main",
            "openloris_rosbag_replay = embodied_slam_tools.replay_node:main",
            "slam_trajectory_recorder = embodied_slam_tools.trajectory_recorder:main",
            "voice_slam_session_orchestrator = embodied_slam_tools.showcase_session_node:main",
        ],
    },
)
