## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.
ROS Noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **serial**
sudo apt-get install ros-noetic-serial

### 1.3. **eigen**
sudo apt-get install libeigen3-dev

## 2. Build
在工作空间的src文件夹中：
* git clone https://gitee.com/fancinnov/fcu_core_v2.git 
* git clone https://gitee.com/fancinnov/quadrotor_msgs.git 
* git clone https://gitee.com/fancinnov/fcu_core_rviz_swarm_goals_plugin.git 
* git clone https://gitee.com/fancinnov/FanciSwarm_urdf.git

在工作空间：catkin_make

## 3. Source
source devel/setup.bash

## 4. 如果用到串口需要配置权限
sudo chmod 777 /dev/ttyACM0

## 5. 运行node
roslaunch fcu_core fcu_core.launch
