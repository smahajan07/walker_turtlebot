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
 *@file walker.hpp
 *@author Sarthak Mahajan
 *@brief Header file for Walker class, contains all member and function
 * declarations.
 */

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
    ros::NodeHandle n;
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
     * @brief checks for the laser scan and whether the robot is close
     *  to an obstacle or not
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