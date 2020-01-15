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
$ sudo apt-get install ros-melodic-??

(Build)
$ cd ~/catkin_ws && catkin_make
```

### Execute ROS packages
- Run below in the terminal window.

```sh
(Bringup)
$ roslaunch ur_rh_bringup ur_rh_bringup.launch

(Gazebo)
$ roslaunch ur_rh_gazebo ur_rh_gazebo.launch

(Gazebo)
# Close gripper
$ rostopic pub -1 /rh_p12_rn_a/rh_p12_rn_a_position/command std_msgs/Float64 "data: 1.1"
# Open gripper
$ rostopic pub -1 /rh_p12_rn_a/rh_p12_rn_a_position/command std_msgs/Float64 "data: 0.0"
```

### Reference
- [universal_robot](https://github.com/ros-industrial/universal_robot)
- [RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A)
