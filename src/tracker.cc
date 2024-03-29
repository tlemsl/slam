#include "slam_practice/tracker.h"

#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

Tracker::Tracker(ros::NodeHandle* nh, std::string image_topic) {
  mF = 389.9472961425781;
  mPrinciplePoint = cv::Point2d(321.9215393066406, 239.0868377685547);

  mpDetector = cv::ORB::create();
  mpDescriptorExtractor = cv::ORB::create();
  mpMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  posePub = nh->advertise<geometry_msgs::PoseArray>("Poses", 1);
  ros::Subscriber image_sub =
      nh->subscribe(image_topic, 1, &Tracker::ImageCallback, this);
  ROS_INFO("Done Tracker setup!");
  ros::spin();
}

void Tracker::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat image;

  // Copy the ros image message to cv::Mat. Convert to grayscale if it is a
  // color image.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  image = cv_ptr->image;

  bool mbRGB = true;
  if (image.channels() == 3) {
    if (mbRGB)
      cvtColor(image, image, cv::COLOR_RGB2GRAY);
    else
      cvtColor(image, image, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    if (mbRGB)
      cvtColor(image, image, cv::COLOR_RGBA2GRAY);
    else
      cvtColor(image, image, cv::COLOR_BGRA2GRAY);
  }
  mpCurrentFrame =
      std::make_shared<Frame>(image, mpDetector, mpDescriptorExtractor);
  if (mpCurrentFrame->mKeypoints.size() < 10) {
    return;
  }
  ROS_INFO("test");

  if (mpLastFrame != nullptr) {
    std::vector<cv::DMatch> matches;
    mpMatcher->match(mpLastFrame->mDescriptors, mpCurrentFrame->mDescriptors,
                     matches);
    ROS_INFO("test1");

    double min_dist = 5, max_dist = 0;

    for (int i = 0; i < mpLastFrame->mDescriptors.rows; i++) {
      double dist = matches[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }

    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < mpLastFrame->mDescriptors.rows; i++) {
      if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
        good_matches.push_back(matches[i]);
      }
    }

    if (good_matches.size() < 10) {
      return;
    }

    // TODO(Minjong) cv::drawMatch makes a problem!
    // cv::Mat img_goodmatch;
    // cv::drawMatches(mpLastFrame->mImage, mpLastFrame->mKeypoints,
    //                 mpCurrentFrame->mImage, mpCurrentFrame->mKeypoints,
    //                 good_matches, img_goodmatch);
    // cv::imshow("Good match", img_goodmatch);
    // cv::waitKey(1);

    cv::Mat essential_matrix, R, t;
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < (int)good_matches.size(); i++) {
      points1.push_back(mpLastFrame->mKeypoints[good_matches[i].queryIdx].pt);
      points2.push_back(
          mpCurrentFrame->mKeypoints[good_matches[i].trainIdx].pt);
    }
    ROS_INFO("test3");

    essential_matrix =
        cv::findEssentialMat(points1, points2, mF, mPrinciplePoint);
    cv::recoverPose(essential_matrix, points1, points2, R, t, mF,
                    mPrinciplePoint);
    mpCurrentFrame->mpPrevFrame = mpLastFrame;
    mpCurrentFrame->setPose(R, t);
    publishPose();
  }

  mpLastFrame = mpCurrentFrame;
}
void Tracker::publishPose() {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();

  geometry_msgs::Pose pose;

  Eigen::Quaterniond Q = mpCurrentFrame->getPose().inverse().unit_quaternion();
  Eigen::Vector3d T = mpCurrentFrame->getPose().inverse().translation();
  pose.position.x = T.x();
  pose.position.y = T.y();
  pose.position.z = T.z();

  pose.orientation.x = Q.x();
  pose.orientation.y = Q.y();
  pose.orientation.z = Q.z();
  pose.orientation.w = Q.w();

  posearray_msg.header = header;
  posearray_msg.poses.push_back(pose);
  posePub.publish(posearray_msg);
}