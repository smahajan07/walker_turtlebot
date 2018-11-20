#include "walker_turtlebot/walker.hpp"

int main(int argc, char **argv) {
  // initialize ros node
  ros::init(argc, argv, "walker");
  // create walker object
  Walker walkerObj;
  // move the robot
  walkerObj.moveRobot();

  return 0;
}