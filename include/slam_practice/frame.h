#ifndef __FRAME__
#define __FRAME__

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

class Frame {
 public:
  Frame(cv::Mat image, cv::Ptr<cv::FeatureDetector> pDetector,
        cv::Ptr<cv::DescriptorExtractor> pDescriptorExtractor);

  cv::Mat mImage;
  std::vector<cv::KeyPoint> mKeypoints;
  cv::Mat mDescriptors;

 private:
  cv::Ptr<cv::FeatureDetector> mpDetector;
  cv::Ptr<cv::DescriptorExtractor> mpDescriptorExtractor;
};

#endif
