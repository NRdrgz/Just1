# Just1 Software

This is a ROS2-based control system for a four-wheeled robot with joystick control.

## References for external ROS packages
- Camera ROS: https://index.ros.org/p/camera_ros/
- Foxglove Bridge: https://github.com/foxglove/ros-foxglove-bridge
- RTAB-Map: https://github.com/introlab/rtabmap_ros

## Installation

1. Install Ubuntu Server 24.04 on your Pi <br>
See `Reads/How_to_Install_Ubuntu_on_Pi.md` <br>
Once you are connected through SSH to your Pi, continue: <br>

2. Install ROS2 on your Pi <br>
See `Reads/How_to_Install_ROS2_on_Pi.md` <br>
We are voluntarily not using a virtual environment because it creates issues with the Camera and with ROS2 <br>

3. Install git and clone this repository on your Pi <br>
```bash
sudo apt install git -y
git clone https://github.com/NRdrgz/Just1.git
cd Just1/Bot/Software
```

4. Install Dependencies: <br>
```bash
sudo apt install python3-pip python3-gpiozero python3-pygame libcap-dev ninja-build libyaml-dev python3-yaml python3-ply python3-jinja2 meson libdrm ros-jazzy-foxglove-bridge python3-smbus i2c-tools ros-jazzy-rtabmap-ros ros-jazzy-imu-filter-madgwick
pip install --break-system-packages -r requirements.txt
```
5. Build libcamera from source <br>
As libcamera is not available on Ubuntu Server 24.04 we need to install it from the source
```bash
cd ~
git clone https://git.libcamera.org/libcamera/libcamera.git
cd libcamera
meson setup build
sudo ninja -C build install
```
Add the directory to Python search path
```bash
echo 'export PYTHONPATH=/usr/local/lib/aarch64-linux-gnu/python3.12/site-packages:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

And we also have to build from source pykms
```bash
cd ~
git clone https://github.com/tomba/kmsxx.git
cd kmsxx
meson setup build
sudo ninja -C build install
```
```bash
echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

6. Connect a Nintendo Lic Pro Controller if you have one <br>
See `Reads/How_to_Connect_Nintendo_Pro_Controller.md` <br>

7. Build the ROS2 packages: <br>
```bash
cd ~/Just1/Bot/Software
colcon build --symlink-install
```

To build a single package you can do <br>
```bash
colcon build --packages-select <package_name> --symlink-install
```

8. Source the workspace: <br>
```bash
source install/setup.bash
```

## Usage in Manual
At this point you can control the robot manually. <br>
To start the manual control system: <br>

```bash
ros2 launch just1_bringup just1.launch.py mode:=manual
```

## Get the Video feed
To see the video captured from the Camera, you can either
- Use Foxglove (preferred) <br>
The foxglove_bridge_node used in just1_bringup is used to select the relevant topics to whitelist to Foxglove <br>
You can then visualize Camera data in a Foxglove UI <br>

- Use the homemade websocket:
Activate the camera_web_socket node in just1.launch.py <br>
Navigate to the Web page in src/just1_camera/web/Webpage.html <br>
Make sure to change the PI ip address in the Webpage <br>

## Continue installation for Autonomous mode



