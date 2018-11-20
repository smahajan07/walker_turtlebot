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
  velPub = n.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 1000);
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
  float min = 0.75;
  for(const auto& dist : msg->ranges) {
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
  // set publisher frequency
  ros::Rate loop_rate(5);
  // move bot if ros is okay
  while(ros::ok()) {
    // check for obstacle
    if(!checkObstacle()) {
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