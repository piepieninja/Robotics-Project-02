assuming you could run the code before, you only need to follow these instructions to run the code:

after sourcing and making, in Terminal 1, run:

export ROBOT_INITIAL_POSE="-x 2 -y 2"

roslaunch Robotics_Project_02 teamC.launch

after sourcing and making, in Terminal 2, run:

rostopic echo /bayes_loc






END OF UPDATED CODE INSTRUCTIONS
Team-C

Robotics Project 02


Set Up:

You should have gazebo_ros installed.

export ROBOT_INITIAL_POSE="-x 2 -y 2"

Place this project in src folder in existing catkin workspace.

To install PR2 omnidirectional robot:

$ sudo apt-get install ros-kinetic-pr2-gazebo
$ rosdep install pr2_gazebo
$ rosmake pr2_gazebo

To install custom model for this project (from Robotics_Project_02) (only need to do this once):

cp -r src/models/pr2Room ~/.gazebo/models




Command to launch project(from workspace) (after catkin_make and source command):

roslaunch Robotics_Project_02 teamC.launch


Gazebo will load with our robot inside 4 walls, enclosing the 100mX100m grid. Our robot starts at position (2,2). You can ignore the sensor lights the robot is shooting. He can’t control himself.


Use the script running in the new xterm window to move the robot:

Control keys (just press, don’t hold; wait for completion message to appear before pressing another command):

Shift+i: forward (increase x)

Shift+,: backward (decrease x)

Shift+j: upward (increase y)

Shift+l: downward (decrease y)

Each command will move the robot in the specified direction with the following motion model:
10% chance of no movement at all
80% chance of intended 1m total movement in intended direction
10% chance of unintended 2m total movement in intended direction
After the command is given, a message will appear in the ‘my_keyboard.py’ xterm window telling you that the movement is complete and you can enter the next command.

The sensor data is calculated with gaussian error (sigma = 0.3) = gaussian_values = [0.0, 0.04779, 0.904419, 0.04779, 0.0]

Topics to consider: localization is published in bayes_loc

CONTRIBUTIONS:
Christian: implemented keyboard script, world, launch file, part of README.md, message file
Caleb: implemented bayes script, sensor script
Shrinidhi: implemented bayes script, sensor script
Kyle: researched robot, implemented bayes script
