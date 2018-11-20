#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Class for Walker
 */

class Walker {
  private:
    // keep track of possible collision
    bool collision;
    // create a object to publish velocities
    geometry_msgs::Twist msg;
    // create a node handle
    ros::Nodehandle n;
    // create a subsciber for the laserscan topic
    ros::Subscriber sub;
    // create a publisher to publish velocities
    ros::Publisher velPub;
    // variables to store linear and angular velocity
    float linVel;
    float angVel;

  public:
    /**
     * @brief constructor
     */
    Walker();
    /**
     * @brief callback function to get laser scan readings
     * 
     * @return None
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr&);
    /**
     * @brief checks for 
     * 
     * @return state of collision
     */
    bool checkObstacle();
    /**
     * @brief Moves the robot forward, if there is no obstacle
     * else, makes a turn in place
     * 
     * @return None
     */
    void moveRobot();
};

#endif  // INCLUDE_WALKER_HPP_