from setuptools import setup

package_name = "just1_camera"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nico",
    maintainer_email="nicolasrdrgzrosdev@protonmail.com",
    description="Camera driver for Just1 robot",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = just1_camera.camera_node:main",
            "camera_websocket = just1_camera.camera_web_socket:main",
        ],
    },
)
