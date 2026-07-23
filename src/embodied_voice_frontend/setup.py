from setuptools import find_packages, setup


package_name = "embodied_voice_frontend"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={
        "fuzzy": ["rapidfuzz>=3.0.0"],
        "webrtc-vad": ["webrtcvad>=2.0.10"],
        # 直接维护 Silero ONNX state/context，不为 VAD 引入完整 PyTorch。
        "silero-vad": ["onnxruntime==1.27.0"],
        "kws": ["openwakeword>=0.6.0"],
        "livekit-kws": ["livekit-wakeword>=0.1.0"],
    },
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Edddddddddy",
    maintainer_email="Edddddddddy@users.noreply.github.com",
    description="Reusable ROS 2 voice frontend adapters for embodied agents",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "silero_vad = embodied_voice_frontend.silero_vad_node:main",
            "webrtc_vad = embodied_voice_frontend.webrtc_vad_node:main",
            "keyword_wake = embodied_voice_frontend.keyword_wake_node:main",
            "speaker_identity = embodied_voice_frontend.speaker_identity_node:main",
        ],
    },
)
