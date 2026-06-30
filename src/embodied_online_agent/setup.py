from glob import glob
from setuptools import find_packages, setup

package_name = "embodied_online_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/prompts", glob("prompts/*.txt")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@example.com",
    description="Streaming online voice agent for an embodied robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "online_agent = embodied_online_agent.online_agent_node:main",
        ],
    },
)
