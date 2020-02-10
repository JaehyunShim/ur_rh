## UR5e + RH-P12-RN

### Install ROS Melodic
```sh
$ sudo apt-get update && sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/rjshim/robotis_engineer/master/install_ros_melodic.sh && chmod 755 ./install_ros_melodic.sh && bash ./install_ros_melodic.sh
```

### Install ROS packages and Build
```sh
(Move to your catkin workspace)
$ cd ~/catkin_ws/src/

(Download packages)
$ git clone https://github.com/ros-industrial/universal_robot.git
$ git clone https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git

(Install binary packages)

(Build)
$ cd ~/catkin_ws && catkin_make
```

### Execute ROS packages
- Run below in the terminal window.

```sh
(Real Hardware)
# Bringup
$ roslaunch ur_rh_bringup ur_rh_bringup.launch
# MoveIt
$ roslaunch ur_rh_moveit_config move_group.launch
$ roslaunch ur_rh_moveit_config moveit_rviz.launch

(Gazebo Simulation)
# Gazebo
$ roslaunch ur_rh_gazebo ur_rh.launch
# MoveIt
$ roslaunch ur_rh_moveit_config move_group.launch
$ roslaunch ur_rh_moveit_config moveit_rviz.launch

# Control gripper
$ rostopic pub -1 /rh_p12_rn_a/rh_p12_rn_a_position/command std_msgs/Float64 "data: 1.1"
# GUI
$ roslaunch ur_rh_gui ur_rh_gui.launch
```

### Reference
- [universal_robot](https://github.com/ros-industrial/universal_robot)
- [RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A)
