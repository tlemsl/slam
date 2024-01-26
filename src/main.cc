#include "slam_practice/node.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Front end");
  ros::NodeHandle nh;
  Node obj(&nh);
  ROS_INFO("Good bye");
  return 0;
}
