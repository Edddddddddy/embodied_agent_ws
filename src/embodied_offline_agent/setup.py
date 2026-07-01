from glob import glob
from setuptools import find_packages, setup

package_name = "embodied_offline_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.txt")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@example.com",
    description="Offline embodied-agent model adapters and asynchronous pipeline",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "offline_agent = embodied_offline_agent.offline_agent_node:main",
        ]
    },
)
