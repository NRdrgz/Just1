from setuptools import setup

package_name = "robot_state_monitor"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={"": "src"},  # This tells setuptools to look in the src directory
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Robot state monitoring and publishing",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "state_publisher_node = robot_state_monitor.state_publisher_node:main",
        ],
    },
)
