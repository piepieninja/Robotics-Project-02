# Robotics Project 02

asdf, yeet

### Set Up

done within existing catkin workspace: `~/catkin_ws/src` . Place this project there.

run: `catkin_make` to generate this project

To install PR2 omnidirectional robot:
```bash
$ sudo apt-get install ros-kinetic-pr2-gazebo
$ rosmake rosdep
$ rosdep install pr2_gazebo
$ rosmake pr2_gazebo
```