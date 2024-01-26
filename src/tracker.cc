#include "slam_practice/tracker.h"

Tracker::Tracker(ros::NodeHandle* nh, std::string image_topic) {
  mpDetector = cv::ORB::create();
  mpDescriptorExtractor = cv::ORB::create();
  mpMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  ros::Subscriber image_sub =
      nh->subscribe(image_topic, 1, &Tracker::ImageCallback, this);
  ROS_INFO("Done Tracker setup!");
  ros::spin();
}

void Tracker::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Get Image!");
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
  ROS_INFO("Convert Image");

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
  if (mpLastFrame != nullptr) {
    std::vector<cv::DMatch> matches;
    mpMatcher->match(mpLastFrame->mDescriptors, mpCurrentFrame->mDescriptors,
                     matches);

    double min_dist = 10000, max_dist = 0;

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
    cv::Mat img_match;
    cv::Mat img_goodmatch;
    cv::drawMatches(mpLastFrame->mImage, mpLastFrame->mKeypoints,
                    mpCurrentFrame->mImage, mpCurrentFrame->mKeypoints, matches,
                    img_match);
    cv::drawMatches(mpLastFrame->mImage, mpLastFrame->mKeypoints,
                    mpCurrentFrame->mImage, mpCurrentFrame->mKeypoints,
                    good_matches, img_goodmatch);
    cv::imshow("Image Match", img_match);
    cv::imshow("Good match", img_goodmatch);
    cv::waitKey(1);
  }
  mpLastFrame = mpCurrentFrame;
}
