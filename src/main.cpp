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
 *@file main.cpp
 *@author Sarthak Mahajan
 *@brief Main program source file that creates and calls the walker class
 * object and runs in obstacle avoidance mode
 */

#include "walker_turtlebot/walker.hpp"

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "walker");
  /**
  * Create an object of the walker class and initiate the movement.
  * The turtlebot keeps moving forward unless it is close to an obstacle,
  * and in that case rotates in place and starts moving straight again. 
  */
  // create walker object
  Walker walkerObj;
  // move the robot
  walkerObj.moveRobot();

  return 0;
}
