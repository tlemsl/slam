#ifndef __TRACKER__
#define __TRACKER__
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>

#include "slam_practice/frame.h"


class Tracker {
 public:
  Tracker(ros::NodeHandle *nh, std::string image_topic);
  void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
  void run();

 private:
  cv::Ptr<cv::DescriptorMatcher> mpMatcher;
  cv::Ptr<cv::FeatureDetector> mpDetector;
  cv::Ptr<cv::DescriptorExtractor> mpDescriptorExtractor;

  std::shared_ptr<Frame> mpLastFrame, mpCurrentFrame;
	std::shared_ptr<Frame> mpInitFrame;
  
  ros::Publisher posePub;
  geometry_msgs::PoseArray posearray_msg;

	// Camera intrinsic
  double mF;
	cv::Point2d mPrinciplePoint;

	std::vector<std::shared_ptr<Frame>> mvpFrame;

  void publishPose();
};

#endif