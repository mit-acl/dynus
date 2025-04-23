#!/bin/bash 

# This will prompt the user for sudo password and the credentials will be cached for 15 minutes
# All subsequent sudo commands won't prompt user for password as it is already cached
sudo -v 

# Go to home directory and create code directory 
cd /home/${USER}
mkdir code

# Basic software
sudo rm -rf /var/lib/apt/lists/*
sudo apt update
sudo apt upgrade -y
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -q -y --no-install-recommends git tmux vim wget tmuxp make openssh-server net-tools g++ xterm python3-pip 
pip install pymavlink
sudo apt install -y libomp-dev libpcl-dev libeigen3-dev

# Install ROS2 Humble
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
echo -e "\n" | sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt install -y ros-humble-desktop 
sudo apt install -y ros-dev-tools 
echo >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
export ROS_DISTRO=humble

# Ros dependencies 
sudo apt-get install -y  ros-${ROS_DISTRO}-octomap 
sudo apt-get install -y  ros-${ROS_DISTRO}-octomap-msgs 
sudo apt-get install -y  ros-${ROS_DISTRO}-octomap-ros 
sudo apt-get install -y  ros-${ROS_DISTRO}-octomap-rviz-plugins 
sudo apt-get install -y  ros-${ROS_DISTRO}-gazebo-* 
sudo apt-get install -y  ros-${ROS_DISTRO}-pcl-conversions 
sudo apt-get install -y  ros-${ROS_DISTRO}-example-interfaces 
sudo apt-get install -y  ros-${ROS_DISTRO}-pcl-ros 
sudo apt-get install -y  ros-${ROS_DISTRO}-rviz2 
sudo apt-get install -y  ros-${ROS_DISTRO}-rqt-gui 
sudo apt-get install -y  ros-${ROS_DISTRO}-rqt-gui-py 
sudo apt-get install -y  ros-${ROS_DISTRO}-tf2-tools 
sudo apt-get install -y  ros-${ROS_DISTRO}-tf-transformations

sudo apt install -y  ros-${ROS_DISTRO}-turtlesim 
sudo apt install -y  ros-${ROS_DISTRO}-rqt* 
sudo apt install -y  ros-${ROS_DISTRO}-rviz2 
sudo apt install -y  ros-${ROS_DISTRO}-gazebo-ros-pkgs 
sudo apt install -y  ros-${ROS_DISTRO}-rviz-common 
sudo apt install -y  libpcl-dev 
sudo apt install -y  build-essential

# Install Gurobi
wget https://packages.gurobi.com/11.0/gurobi11.0.3_linux64.tar.gz -P . 
tar -xzf gurobi11.0.3_linux64.tar.gz
rm gurobi11.0.3_linux64.tar.gz
sudo mv gurobi1103/ /opt

cd /opt/gurobi1103/linux64/src/build 
make && cp libgurobi_c++.a ../../lib/
echo >> ~/.bashrc
echo "# Gurobi" >> ~/.bashrc
echo 'export GUROBI_HOME="/opt/gurobi1103/linux64" ' >> ~/.bashrc
echo 'export PATH="${PATH}:${GUROBI_HOME}/bin" ' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="${GUROBI_HOME}/lib" ' >> ~/.bashrc
source ~/.bashrc 

export GUROBI_HOME="/opt/gurobi1103/linux64" 
export PATH="${PATH}:${GUROBI_HOME}/bin" 
export LD_LIBRARY_PATH="${GUROBI_HOME}/lib" 

# Dynus and dependencies
mkdir -p /home/${USER}/code/dynus_ws/src
cd /home/${USER}/code/dynus_ws/src
git https://github.com/XXXXXXXXX/dynus.git
git clone https://github.com/XXXXXXXXX/dynus_interfaces.git
git clone https://github.com/XXXXXXXXX/octomap_mapping.git
git clone https://github.com/XXXXXXXXX/realsense_gazebo_plugin.git
git clone https://github.com/XXXXXXXXX/livox_laser_simulation_ros2.git
git clone https://github.com/XXXXXXXXX/octomap_rviz_plugins.git
git clone https://github.com/XXXXXXXXX/gazebo_ros_pkgs.git

mkdir -p /home/${USER}/code/decomp_ws/src
cd /home/${USER}/code/decomp_ws/src
git clone https://github.com/XXXXXXXXX/DecompROS2.git
mkdir -p /home/${USER}/code/livox_ws/src
cd /home/${USER}/code/livox_ws/src
git clone https://github.com/XXXXXXXXX/livox_ros_driver2.git
cd /home/${USER}/code
git clone https://github.com/Livox-SDK/Livox-SDK2.git

# Build workspace 
#decomp 
cd /home/${USER}/code/decomp_ws
source /opt/ros/humble/setup.sh && colcon build --packages-select decomp_util
source /home/${USER}/code/decomp_ws/install/setup.sh && source /opt/ros/humble/setup.sh && colcon build

#Livox-SDK2
cd /home/${USER}/code/Livox-SDK2
mkdir build 
cd /home/${USER}/code/Livox-SDK2/build 
cmake .. && make -j && sudo make install 

#livox_ros_drver2
cd /home/${USER}/code/livox_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh && ./build.sh humble

#Dynus 
cd /home/${USER}/code/dynus_ws
source /opt/ros/humble/setup.sh 
source /home/${USER}/code/decomp_ws/install/setup.sh 
export CMAKE_PREFIX_PATH=/home/${USER}/code/livox_ws/install/livox_ros_driver2:/home/${USER}/code/decomp_ws/install/decomp_util
colcon build

# Add livox to library path 
echo >> ~/.bashrc
echo >> 'DYNUS ENVIRONMENT SETUP' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/home/${USER}/code/livox_ws/install/livox_ros_driver2/lib:${LD_LIBRARY_PATH}" ' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH}" ' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH="/home/${USER}/code/decomp_ws/install/decomp_ros_msgs/lib:${LD_LIBRARY_PATH}" ' >> ~/.bashrc
echo >> 'END DYNUS ENVIRONMENT SETUP' >> ~/.bashrc
source ~/.bashrc

# Install and configure cyclonedds
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

