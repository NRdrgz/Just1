from setuptools import setup, find_packages

package_name = "robot_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[],  # Explicitly set to empty list
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/robot.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nico",
    maintainer_email="nicolasrdrgzrosdev@protonmail.com",
    description="Launch files and configurations for bringing up the robot",
    license="Apache License 2.0",
    tests_require=["pytest"],
)
