# Just1 Software

This is a ROS2-based control system for a four-wheeled robot with joystick control.


## Installation

1. Install Ubuntu Server 24.04 on your Pi <br>
See Reads/How_to_Install_Ubuntu_on_Pi.md <br>
Once you are connected through SSH to your Pi, continue: <br>

2. Install ROS2 on your Pi <br>
See Reads/How_to_Install_ROS2_on_Pi.md <br>
We are voluntarily not using a virtual environment because it creates issues with the Camera and with ROS2 <br>

3. Install git and clone this repository on your Pi <br>
```bash
sudo apt install git -y
git clone https://github.com/NRdrgz/Just1.git
cd Just1/Bot/Software
```

4. Install Python dependencies: <br>
```bash
sudo apt install python3-pip python3-gpiozero python3-pygame
pip install -r requirements.txt
```

5. Connect a Nintendo Lic Pro Controller if you have one <br>
See Reads/How_to_Connect_Nintendo_Pro_Controller.md <br>

6. Build the packages: <br>
```bash
colcon build --symlink-install
```

To build a single package you can do <br>
```bash
colcon build --packages-select <package_name> --symlink-install
```

7. Source the workspace: <br>
```bash
source install/setup.bash
```

## Usage

To start the manual control system: <br>

```bash
ros2 launch just1_bringup just1.launch.py mode:=manual
```
