#include "slam_practice/node.h"
#include "slam_practice/tracker.h"

Node::Node(ros::NodeHandle *nh){
  std::string image_topic;
  ros::param::get("~image_topic", image_topic);
  ROS_INFO("Image topic is %s", image_topic.c_str());
  Tracker tracker(nh, image_topic);

};
