# Robotics Project 02

asdf, yeet

### Set Up

done within existing catkin workspace: `~/catkin_ws/src` . Place this project there.

run: `catkin_make` to generate this project

To install PR2 omnidirectional robot:
```bash
$ sudo apt-get install ros-kinetic-pr2-gazebo
$ rosdep install pr2_gazebo
$ rosmake pr2_gazebo
```
To run (link: http://wiki.ros.org/pr2_simulator/Tutorials) :
```bash
$ roslaunch gazebo_ros empty_world.launch
$ roslaunch pr2_gazebo pr2.launch
```
<<<<<<< HEAD
To move install pr2_teleop (link: http://wiki.ros.org/pr2_simulator/Tutorials/WorkingWithGazeboOverRos):
```bash
sudo apt-get install ros-kinetic-pr2-teleop
```

Then type:
```bash
$ roslaunch pr2_teleop teleop_keyboard.launch
```

Control keys:
Use 'WASD' to translate
Use 'QE' to yaw
Press 'Shift' to run
=======

### Topics

http://docs.ros.org/diamondback/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html

this is `/robot_pose_ekf/odom_combined`
>>>>>>> 8b2fc8220f3391b770dc717065df03d519612901

Christian's commands:

(From Robotics_Project_02)

cp -r src/models/pr2Room ~/.gazebo/models

roslaunch Robotics_Project_02 pr2Room.launch

ROBOT_INITIAL_POSE="-x 2 -y 2" roslaunch pr2_gazebo pr2_no_arms.launch

rosrun Robotics_Project_02 my_keyboard.py
