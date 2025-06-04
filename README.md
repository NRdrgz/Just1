# Just1 - An Open-Source Mecanum Wheel Robot

Just1 is an open-source robotics platform designed for learning, experimentation, and fun! Built around a Raspberry Pi and equipped with mecanum wheels, this robot offers both manual control and autonomous navigation capabilities.

## Project Goals

Just1 was created with several objectives in mind:
- Provide a fun and engaging robotics platform
- Develop practical robotics and programming skills
- Share knowledge and encourage others to build their own robots
- Create an accessible platform for learning and experimentation

## Workspace Organization

### Hardware
Located in `/Bot/Hardware`:
- Bill of Materials (BoM)
- SolidWorks design files
- Assembly instructions and wiring diagrams

### Software
Located in `/Bot/Software`:
- ROS2 packages
- Installation guides
- Manual control documentation

## Current Features

- Complete hardware design with BoM and SolidWorks files
- Comprehensive software documentation for Raspberry Pi setup
- Manual control using Nintendo Controller
- Real-time camera feed accessible via homemade web interface or through Foxglove

## Features in Progress

- Autonomous navigation using ROS2 VSLAM (RTAB Map)
- Add documentation on Camera feedback through websocket

## Short-term To-Do List

- Create detailed README for Hardware folder
- Add assembly and wiring documentation
- Add control and camera feedback on phone
- Use Foxglove CompressedVideo instead of CompressedImage for Camera feedback

## Long-term Goals and Potential Improvements

- Add a Docker Image for easier and faster deployment
- Perform Navigation through Neural Network architecture instead of VSLAM 
- Replace AA batteries with voltage booster for simplified power management
- Integrate LIDAR system, IMU and/or wheel encoders for advanced navigation and experimentation
- Integrate memory / latency profilers to optimize processes and detect memory leaks



