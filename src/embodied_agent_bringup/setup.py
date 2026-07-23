from setuptools import find_packages, setup


package_name = "embodied_agent_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Edddddddddy",
    maintainer_email="Edddddddddy@users.noreply.github.com",
    description="Reusable launch contracts for the embodied agent stack",
    license="Apache-2.0",
    tests_require=["pytest"],
)
