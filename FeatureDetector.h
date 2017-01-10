#ifndef SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
#define SOFA_OR_PROCESSOR_FEATUREDETECTOR_H

#include "DetectorOptions.h"
#include "initplugin.h"

#include <SofaORCommon/Image.h>

#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FeatureDetector : public core::DataEngine
{
  enum DetectorType
  {
    FAST = 0,
    MSER = 1,
    ORB = 2,
    BRISK = 3,
    KAZE = 4,
    AKAZE = 5,
    SIFT = 6
  };

 public:
  SOFA_CLASS(FeatureDetector, core::DataEngine);

 public:
  FeatureDetector();
  virtual ~FeatureDetector();

  void init();
  void update();
  void reinit();

  Data<common::Image> d_image;
  Data<common::Image> d_mask;
  Data<sofa::helper::vector<cv::KeyPoint> > d_keypoints;
  Data<sofa::helper::OptionsGroup> d_detectorType;

  void handleEvent(sofa::core::objectmodel::Event* event);


private:
  BaseOpts* m_detector;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
