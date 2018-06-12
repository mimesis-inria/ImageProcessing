ifndef SOFACV_FEATURES_FEATUREDETECTOR_H
#define SOFACV_FEATURES_FEATUREDETECTOR_H

#include "Detectors.h"
#include "common/ImageFilter.h"

#include <SofaCV/SofaCV.h>

#include <sofa/core/DataTracker.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace features
{
class SOFA_IMAGEPROCESSING_API FeatureDetector : public common::ImageFilter
{
  enum DetectorMode
  {
    DETECT_ONLY,
    COMPUTE_ONLY,
    DETECT_AND_COMPUTE
  };

  enum DetectorType
  {
    FAST,
    MSER,
    ORB,
    BRISK,
    KAZE,
    AKAZE,
    BLOB,
#ifdef SOFAOR_OPENCV_CONTRIB_ENABLED
    BRIEF,
    SIFT,
    SURF,
    DAISY,
#endif // SOFAOR_OPENCV_CONTRIB_ENABLED
    DetectorType_COUNT
  };

 public:
  SOFA_CLASS(FeatureDetector, common::ImageFilter);

 public:
  FeatureDetector();
  virtual ~FeatureDetector() override;

  virtual void init() override;
  virtual void reinit() override;
  virtual void Update() override;
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug) override;

	sofa::Data<sofa::helper::OptionsGroup> d_detectMode;
    sofa::Data<cvMat> d_mask;
	sofa::Data<sofa::helper::OptionsGroup> d_detectorType;
    sofa::Data<sofa::helper::vector<cvKeypoint> > d_keypoints;
    sofa::Data<cvMat> d_descriptors;

 protected:
  void detectTypeChanged();
  void detectModeChanged();

 private:
  BaseDetector* m_detectors[DetectorType_COUNT];

  std::vector<cv::KeyPoint> _v;
  cvMat _d;
};

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_FEATUREDETECTOR_H
