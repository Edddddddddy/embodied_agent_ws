from glob import glob

from setuptools import find_packages, setup


package_name = "embodied_agent_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.json"),
        ),
        ("share/" + package_name + "/prompts", glob("prompts/*.txt")),
        ("share/" + package_name + "/knowledge", glob("knowledge/*.md")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Edddddddddy",
    maintainer_email="Edddddddddy@users.noreply.github.com",
    description="Shared domain, orchestration, and ROS transport modules for Agent adapters",
    license="Apache-2.0",
)
