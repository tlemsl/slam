#ifndef __FRAME__
#define __FRAME__
#define FMT_HEADER_ONLY
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "sophus/se3.hpp"
#include <vector>

class Frame {
 public:
  Frame(cv::Mat image, cv::Ptr<cv::FeatureDetector> pDetector,
        cv::Ptr<cv::DescriptorExtractor> pDescriptorExtractor);
  Sophus::SE3<double> getPose();
  void setPose(cv::Mat R, cv::Mat t);

  cv::Mat mImage;
  std::vector<cv::KeyPoint> mKeypoints;
  cv::Mat mDescriptors;
  std::shared_ptr<Frame> mpPrevFrame;


 private:
  cv::Ptr<cv::FeatureDetector> mpDetector;
  cv::Ptr<cv::DescriptorExtractor> mpDescriptorExtractor;

  Sophus::SE3<double> mTcw;
};

#endif
