#include "slam_practice/frame.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <iostream>

Frame::Frame(cv::Mat image, cv::Ptr<cv::FeatureDetector> pDetector,
             cv::Ptr<cv::DescriptorExtractor> pDescriptorExtractor)
    : mImage(image),
      mpDetector(pDetector),
      mpDescriptorExtractor(pDescriptorExtractor) {
  // Extract feature points
  mpDetector->detect(mImage, mKeypoints);
  mpDescriptorExtractor->compute(mImage, mKeypoints, mDescriptors);
}

void Frame::setPose(cv::Mat _R, cv::Mat _t) {
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  for (int i = 0; i < 3; i++) {
    T(i) = _t.at<double>(i, 0);
    for (int j = 0; j < 3; j++) R(i, j) = _R.at<double>(i, j);
  }
  if(mpPrevFrame != nullptr) {
    mTcw = Sophus::SE3<double>(R, T) * mpPrevFrame->getPose();
  }
  else {
    mTcw = Sophus::SE3<double>(R, T);
  }
}

Sophus::SE3<double> Frame::getPose() {return mTcw;}
