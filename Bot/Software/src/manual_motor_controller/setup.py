from setuptools import setup, find_packages

package_name = "manual_motor_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/msg", ["msg/WheelSpeeds.msg"]),
    ],
    install_requires=["setuptools", "rosidl_runtime_py"],
    zip_safe=True,
    maintainer="Nico",
    maintainer_email="nicolasrdrgzrosdev@protonmail.com",
    description="Manual motor controller for robot movement",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_node = manual_motor_controller.controller_node:main",
        ],
    },
)
