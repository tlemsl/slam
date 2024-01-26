#include "slam_practice/frame.h"

Frame::Frame(cv::Mat image, cv::Ptr<cv::FeatureDetector> pDetector,
             cv::Ptr<cv::DescriptorExtractor> pDescriptorExtractor)
    : mImage(image),
      mpDetector(pDetector),
      mpDescriptorExtractor(pDescriptorExtractor) {
  // Extract feature points
  mpDetector->detect(mImage, mKeypoints);
  mpDescriptorExtractor->compute(mImage, mKeypoints, mDescriptors);
}
