#ifndef SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
#define SOFA_OR_PROCESSOR_FEATUREDETECTOR_H

#include "Detectors.h"
#include "initplugin.h"

#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>

#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

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
    SIFT = 6,
    BRIEF = 7,
    DetectorType_COUNT
  };

 public:
  SOFA_CLASS(FeatureDetector, core::DataEngine);

 public:
  FeatureDetector();
  virtual ~FeatureDetector();

  void init();
  void update();
  void reinit();

  Data<bool> d_detect;
  Data<common::cvMat> d_image;
  Data<common::cvMat> d_mask;
  Data<sofa::helper::OptionsGroup> d_detectorType;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypoints;
  Data<common::cvMat> d_descriptors;

  void handleEvent(sofa::core::objectmodel::Event* event);

 private:
  BaseDetector* m_detectors[DetectorType_COUNT];
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
