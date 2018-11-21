# Walker Turtlebot

## Overview

This is a custom ROS package for moving the turtlebot around in "Roomba" like mode, i.e. without hitting obstacles. The robot keeps moving forward unless it encounters an obstacle, and when it does it rotates in place and keeps moving forward again. It is a simple implementation of behaviour and can be modified in the future to incorporate more complex behaviour.

## Dependencies
* Ubuntu Xenial (16.04)
* ROS Kinetic

## Build Instructions
* If you already have a catkin workspace then:
```
cd <catkin workspace>
cd src
git clone https://github.com/smahajan07/walker_turtlebot.git
cd ..
catkin_make
source devel/setup.bash
```

* If you do not have a catkin workspace, you can create one by following:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/smahajan07/walker_turtlebot.git
cd ..
catkin_make
```

## Run Instructions (Using Launch File)
```
cd <your catkin workspace>
source devel/setup.bash
roslaunch walker_turtlebot launchWalker.launch
```
To enable rosbag recording (all topics except camera) use this command instead of the previous roslaunch
```
roslaunch walker_turtlebot launchWalker.launch record:=true
```

## Rosbags

* Inspect

Along with the package a sample ROS bag was provided ```results/walker_turtlebot.bag```. 
If you wish to look at the topics in the rosbag and find out more details about it:
```
cd <your catkin workspace>
source devel/setup.bash
cd src/walker_turtlebot/results
rosbag info walker_turtlebot.bag
```
* Play

You can play the above mentioned rosbag by running the following in two different terminals:
```
roscore
```
 and
```
rosbag  play walker_turtlebot.bag
```

