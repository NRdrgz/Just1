from setuptools import setup

package_name = "robot_state_publisher"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your-email@example.com",
    description="Publishes robot state information",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_publisher_node = robot_state_publisher.state_publisher_node:main",
        ],
    },
)
