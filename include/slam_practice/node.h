#ifndef __NODE__
#define __NODE__
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>

#include "slam_practice/tracker.h"

class Node {
 public:
  Node(ros::NodeHandle *nh);

 private:
  Tracker *tracker;
};

#endif
