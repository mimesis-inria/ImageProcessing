#ifndef SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
#define SOFA_OR_PROCESSOR_FEATUREDETECTOR_H

#include "Detectors.h"
#include "core/ImageFilter.h"

#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>

#include <sofa/core/DataTracker.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FeatureDetector : public ImageFilter
{
  enum DetectorMode
  {
    DETECT_ONLY,
    COMPUTE_ONLY,
    DETECT_AND_COMPUTE
  };

  enum DetectorType
  {
    FAST = 0,
    MSER = 1,
    ORB = 2,
    BRISK = 3,
    KAZE = 4,
    AKAZE = 5,
    SIFT = 6,
    SURF = 7,
    BRIEF = 8,
    DAISY = 9,
		BLOB = 10,
		DetectorType_COUNT
  };

 public:
  SOFA_CLASS(FeatureDetector, ImageFilter);

 public:
  FeatureDetector();
  virtual ~FeatureDetector();

  void init();
  virtual void update();
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug);

  Data<sofa::helper::OptionsGroup> d_detectMode;
  Data<common::cvMat> d_mask;
  Data<sofa::helper::OptionsGroup> d_detectorType;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypoints;
  Data<common::cvMat> d_descriptors;

 protected:
  void detectTypeChanged(core::objectmodel::BaseData*);
  void detectModeChanged(core::objectmodel::BaseData*);

 private:
  BaseDetector* m_detectors[DetectorType_COUNT];
  core::DataTracker m_dataTracker;

  std::vector<cv::KeyPoint> _v;
  common::cvMat _d;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
