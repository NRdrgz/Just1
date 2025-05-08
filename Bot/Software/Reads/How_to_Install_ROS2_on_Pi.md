# Installing ROS2 on Ubuntu
Following tutorial here https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

## Check locale support UTF-8
```bash
locale # check for UTF-8
# if not 
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## Add the ROS 2 APT repository
```bash
# Install helper tools to manage repositories
sudo apt install software-properties-common
# Enables the universe repo, where most packages live
sudo add-apt-repository universe
# Install curl to get files from the internet
sudo apt update && sudo apt install curl -y
# Download the official ROS GPG key to verify the packages and save it in a secure place
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Add the address of the ROS 2 repository to our list of addresses 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Install ROS 2
```bash
# Refresh the package list now the the new ROS source has been added
sudo apt update
# Install the ROS Jazzy base
sudo apt install ros-jazzy-ros-base
```

## Source ROS 2 in the terminal 
```bash
# Add the ROS 2 environment setup to the bash so it loads automatically everytime you open a terminal
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install colcon tools
```bash
# Install colcon, the ROS 2 build tool
sudo apt install python3-colcon-common-extensions
```