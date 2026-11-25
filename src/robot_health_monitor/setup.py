from setuptools import find_packages, setup

package_name = "robot_health_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.com",
    description="Robot health monitoring and controlled dropout tests",
    license="MIT",
    entry_points={
        "console_scripts": [
            "monitor = robot_health_monitor.monitor:main",
        ],
    },
)
