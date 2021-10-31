# Tutorial: Installation of clover simulation on Gazebo

## Pre-requisites
1. Ubuntu 20.04
2. ROS Noetic installed and with a configured workspace

## References
This tutorial is based on https://clover.coex.tech/en/simulation_native.html

## Installation
### 1. Open ROS workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2. Clone the repositories
### Clover repository
```bash
git clone --depth 1 https://github.com/CopterExpress/clover
```

### Clover repository for led msgs
```bash
git clone --depth 1 https://github.com/CopterExpress/ros_led
```

### Repository for MAV common messages in ROS
```bash
git clone --depth 1 https://github.com/ethz-asl/mav_comm
```

### 3. Update rosdep
```bash
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt
```
