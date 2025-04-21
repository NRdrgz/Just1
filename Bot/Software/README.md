# Just1 Software

This is a ROS2-based control system for a four-wheeled robot with joystick control.



## Installation
[//]: <> (TODO: add part about Bluetooth controller)


1. Create a virtual env
   ```bash
   python -m venv just1-dev
   source just1-dev/bin/activate 
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

   Set Python path manually so that ROS can use packages from the venv
   ```bash
   export PYTHONPATH=/path/to/your/venv/lib/python3.x/site-packages:$PYTHONPATH
   ```
   You can add it to your shell config file for it to be applied at opening
   ```bash
   nano ~/.bashrc
   ```
   and add the export line at the end


3. Build the packages:
   ```bash
   colcon build --symlink-install
   ```

   To build a single package you can do
   ```bash
   colcon build --packages-select <package_name> --symlink-install
   ```


4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

To start the manual control system:

```bash
ros2 launch just1_bringup just1.launch.py mode:=manual
```
