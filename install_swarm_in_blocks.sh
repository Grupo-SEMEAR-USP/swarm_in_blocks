echo "--- Installing Swarm in Blocks"
sleep 1

echo "--- Updating apt"
sleep 1
sudo apt-get update

echo "--- Installing required tools"
sleep 1
sudo apt install build-essential git python3-pip python3-rosdep

echo "--- Installing ROS"
sleep 1
sudo apt-get install -y curl
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get update; apt-get install -y python3-pip python3-rosdep python3-rosinstall-generator python3-wstool build-essential ros-noetic-desktop

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo "--- Updating rosdep"
sleep 1
sudo -E sh -c 'rosdep init'
rosdep update

echo "--- Creating Catkin workspace"
sleep 1
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo "--- Getting Swarm In Blocks sources ---"
sleep 1
cd ~/catkin_ws/src
git clone https://github.com/Grupo-SEMEAR-USP/swarm_in_blocks.git

echo "--- Installing Swarm in Blocks dependencies"
/usr/bin/python3 -m pip install -r ~/catkin_ws/src/swarm_in_blocks/requirements.txt
# principal libraries for wxPython
sudo apt install -y freeglut3 freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev libgstreamer-plugins-base1.0-dev libgtk-3-dev \
    libjpeg-dev libnotify-dev libsdl2-dev libsm-dev libtiff-dev libwebkit2gtk-4.0-dev libxtst-dev python3-dev libhdf5-dev build-essential \
    python3-venv


echo "--- Getting Clover sources"
sleep 1
cd ~/catkin_ws/src
git clone --depth 1 https://github.com/Grupo-SEMEAR-USP/clover.git
git clone --depth 1 https://github.com/CopterExpress/ros_led
git clone --depth 1 https://github.com/ethz-asl/mav_comm

echo "--- Installing Clover's Python dependencies"
sleep 1
/usr/bin/python3 -m pip install -r ~/catkin_ws/src/clover/clover/requirements.txt

echo "--- Downloading PX4"
sleep 1
git clone --recursive --depth 1 --branch v1.12.0 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/

echo "--- Installing PX4 dependencies"
sleep 1
~/PX4-Autopilot/Tools/setup/ubuntu.sh
pip3 install --user toml

echo "--- Installing dependencies with rosdep"
sleep 1
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y

echo "--- Addding Clover airframe"
sleep 1
ln -s ~/catkin_ws/src/clover/clover_simulation/airframes/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

echo "--- Installing geographiclib datasets"
sleep 1
sudo -E sh -c '/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh'

echo "--- Building the workspace (it takes 10~20min)"
sleep 10
cd ~/catkin_ws
catkin_make

echo "--- Installing QGroundControl"
sleep 1
sudo -E sh -c "usermod -a -G dialout $USER"
sudo -E sh -c 'apt-get remove -y modemmanager; apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav'
curl https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -o ${HOME}/QGroundControl.AppImage
chmod a+x ${HOME}/QGroundControl.AppImage

echo "--- Installing Firefox web browser"
sleep 1
sudo apt-get update; apt-get install -y firefox

echo "--- Installing apache webserver"
sleep 1
sudo apt install apache2

echo "--- Configure apache webserver to serve .ros"
sleep 1
sudo sed -i 's@/var/www/html@'"${HOME}"'/.ros/www@' /etc/apache2/sites-available/000-default.conf
sudo sed -i 's@/var/www/@'"${HOME}"'/.ros/www/@' /etc/apache2/apache2.conf
sudo 000-default.conf
service apache2 reload

echo "--- Cleaning up"
sleep 1
sudo apt-get -y autoremove; apt-get -y autoclean; apt-get -y clean; fstrim -v /

echo "--- Validating"
# python --version # python-is-python3
python2 --version
python3 --version
# ipython --version
ipython3 --version
gazebo --version || true # FIXME: Gazebo exits with 255 on --version somehow
# node -v
# npm -v
# byobu --version
git --version
# vim --version
pip --version
pip3 --version
# monkey --version
systemctl --version
# TODO: add Python tests

roscore -h
rosversion px4
rosversion clover
rosversion aruco_pose
rosversion mavros
rosversion mavros_extras
rosversion ws281x
rosversion led_msgs
rosversion dynamic_reconfigure
rosversion tf2_web_republisher
# rosversion compressed_image_transport
# rosversion rosbridge_suite
rosversion cv_camera
rosversion web_video_server
rosversion nodelet

echo "Trying running the Gazebo simulator, check the output"
timeout --preserve-status 30 roslaunch swarm_in_blocks simulation.launch num:=2 new_blocks_page:=true
