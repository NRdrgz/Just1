from setuptools import setup, find_packages

package_name = "just1_motors"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nico",
    maintainer_email="nicolasrdrgzrosdev@protonmail.com",
    description="Motors for robot movement.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "manual_controller_node = just1_motors.manual_controller_node:main",
            "keyboard_controller_node = just1_motors.keyboard_controller_node:main",
        ],
    },
)
