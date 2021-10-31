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

### 2. Clone clover repositories
### 2.1. Clover most important repository
```bash
git clone --depth 1 https://github.com/CopterExpress/clover
```

### 2.2. Clover repository for led msgs
```bash
git clone --depth 1 https://github.com/CopterExpress/ros_led
```

### 2.3. Repository for MAV common messages in ROS
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

### 4. Clone PX4 repository inside ros workspace
```bash
cd ~/catkin_ws/src
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
```
### Make a symbolic link to the PX4 repo. 

> It might be easy to clover while acess the PX4 path

```bash
ln -s ~/PX4-Autopilot ~/catkin_ws/src/PX4-Autopilot
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/sitl_gazebo
```

### 5. Install PX4 dependencies

PX4 has it owns script for dependencies installation. By default it installs an ARM toolchain to compile PX4 scripts to the flight controller. But we can ignore it by passing the --no-nuttx argument.

#### 5.1 Without ARM toolchain (only simulation):

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh --no-nuttx
```

#### 5.2 With ARM toolchain (full installation):

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

### 6. Add clover frame to PX4 simulator

> It creates a symbolic link from the clover airframes directory to the PX4 airframes directory

```bash
ln -s "$(catkin_find clover_simulation airframes)"/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

### 7. Install geographiclib
 
 > Geographic Library has tools for coordinates conversion. Learn more on https://geographiclib.sourceforge.io/html/python/index.html .
 
```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 8. Build everything

```bash
cd ~/catkin_ws
catkin_make
```


