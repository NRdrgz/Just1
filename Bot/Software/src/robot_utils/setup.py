from setuptools import setup

package_name = "robot_utils"

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
    maintainer="Nico",
    maintainer_email="nicolasrdrgzrosdev@protonmail.com",
    description="Common utilities for robot control",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
