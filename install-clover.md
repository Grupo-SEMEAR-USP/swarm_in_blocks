# Tutorial: Installation of clover simulation on Gazebo

## Pre-requisites
**1.** Ubuntu 20.04   **2.** Python 3   **3.** Pip 3   **4.** ROS Noetic installed and with a configured workspace

## References
This tutorial is based on https://clover.coex.tech/en/simulation_native.html

## Installation

### 0. Update and Upgrade apt
```bash
sudo apt update 
sudo apt upgrade
```
### 1. Open ROS workspace

> Assuming that ~/catkin_ws is your workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2. Clone clover repositories
#### 2.1. Clover most important repository
```bash
git clone --depth 1 https://github.com/CopterExpress/clover
```

#### 2.2. Clover repository for led msgs
```bash
git clone --depth 1 https://github.com/CopterExpress/ros_led
```

#### 2.3. Repository for MAV common messages in ROS
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

> If any error occurs, the dependencies will not be installed.

### 4. Clone PX4 repository inside ros workspace
```bash
cd ~/catkin_ws/src
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
```
### 5. Make a symbolic link to the PX4 repo. 

> It might take it easy to clover while acessing the PX4 directory.

```bash
ln -s ~/PX4-Autopilot ~/catkin_ws/src/PX4-Autopilot
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/sitl_gazebo
```

### 6. Install PX4 dependencies

> PX4 has it owns script for dependencies installation. By default it installs an ARM toolchain to compile PX4 scripts to the flight controller. But we can ignore it by passing the --no-nuttx argument.

#### 6.1 Without ARM toolchain (only Gazebo simulation):
 
 > Recommended with you doesn't have a Flight Controller.

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh --no-nuttx
```

> Note: only run one o these
#### OPTION B: With ARM toolchain (full installation, DONT RUN IF YOU DONT NEED ARM TOOLCHAIN):

```bash
cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```

### 7. Add clover frame to PX4 simulator

> It creates a symbolic link from the clover airframes directory to the PX4 airframes directory

```bash
ln -s "$(catkin_find clover_simulation airframes)"/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

### 8. Install geographiclib
 
 > Geographic Library has tools for coordinates conversion. Learn more on https://geographiclib.sourceforge.io/html/python/index.html .
 
```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 9. Install toml (It causes erros and doesn't has been provided on the installations guides) 

```bash
pip3 install toml
```
Or, if your *python* command refers to Python 3 
```bash
pip install toml
```
### 10. Build everything

```bash
cd ~/catkin_ws
catkin_make
```
> It will take ~20 to 30 min. 


### 11. Run gazebo simulation

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch clover_simulation simulator.launch
```
> Gazebo might run and you'll see Clover quadcopter inside a Aruco map.

## 12. Simple Offboard flight.

```python
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight') # 'flight' is name of your ROS node

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Navigate
navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
```

# Next steps:

> Usage and configuration of the simulation: https://clover.coex.tech/en/simulation_usage.html

> Simple programming: https://clover.coex.tech/en/simple_offboard.html
