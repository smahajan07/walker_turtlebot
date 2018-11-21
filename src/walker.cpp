/**MIT License

Copyright (c) 2018 Sarthak Mahajan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 *@copyright Copyright (c) 2018 Sarthak Mahajan
 *@file walker.cpp
 *@author Sarthak Mahajan
 *@brief Source file for Walker class, contains all function definitions
 * 
 */

#include <iostream>
#include "walker_turtlebot/walker.hpp"

Walker::Walker() {
  // initialize data members
  linVel = 0.1;
  angVel = 1.0;
  collision = false;
  // subscribe to /scan topic call callback function
  sub = n.subscribe <sensor_msgs::LaserScan> ("/scan", 1000,
    &Walker::laserCallback, this);
  // publish the velocity
  velPub = n.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity"
    , 1000);
  // initial msg
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // publish initial message
  velPub.publish(msg);
}

void Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // set value for minimum distance from obstacle
  float min = 0.75;
  // check for collision
  for (const auto& dist : msg->ranges) {
    if (dist < min) {
      collision = true;
      return;
    }
  }
  collision = false;
}

bool Walker::checkObstacle() {
  return collision;
}

void Walker::moveRobot() {
  // sleep for 4 seconds while gazebo starts
  // Could be slow in VM
  ros::Duration(4).sleep();
  // set publisher frequency
  ros::Rate loop_rate(5);
  // move bot if ros is okay
  while (ros::ok()) {
    // check for obstacle
    if (!checkObstacle()) {
      ROS_INFO_STREAM("Moving forward ...");
      // make sure angular velocity is zero
      msg.angular.z = 0.0;
      // move forward
      msg.linear.x = linVel;
    } else {
      ROS_INFO_STREAM("Obstacle encountered. Running away ...");
      // stop the robot
      msg.linear.x = 0.0;
      // turn in place
      msg.angular.z = angVel;
    }
    // publish velocity message
    velPub.publish(msg);
    // spin once and keep running
    ros::spinOnce();
    loop_rate.sleep();
  }
}
