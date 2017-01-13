#ifndef SOFA_OR_PROCESSOR_DETECTORS_H
#define SOFA_OR_PROCESSOR_DETECTORS_H

#include <SofaORCommon/cvMat.h>

#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vector.h>

#include <opencv2/xfeatures2d.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FeatureDetector;

struct BaseDetector
{
  virtual ~BaseDetector();

  virtual void toggleVisible(bool) = 0;

  virtual void detect(const common::cvMat&, const common::cvMat&,
                      std::vector<cv::KeyPoint>&);
  virtual void compute(const common::cvMat& img,
                       std::vector<cv::KeyPoint>& keypoints,
                       common::cvMat& descriptors);

 protected:
  cv::Ptr<cv::Feature2D> m_detector;
};
struct FASTDetector: BaseDetector
{
  FASTDetector(FeatureDetector* c);
  void toggleVisible(bool);
  virtual void compute(const common::cvMat&,
                       std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_warning("FASTDetector::detectAndCompute()")
        << "FAST is detectOnly. descriptors won't be computed.";
  }

  Data<int> threshold;
  Data<bool> nonmaxsuppression;
  Data<sofa::helper::OptionsGroup> type;
};
struct MSERDetector : BaseDetector
{
  MSERDetector(FeatureDetector* c);
  void toggleVisible(bool);
  virtual void compute(const common::cvMat&,
                       std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_warning("MSERDetector::detectAndCompute()")
        << "MSER is detectOnly. descriptors won't be computed.";
  }

  Data<int> delta;
  Data<int> minArea;
  Data<int> maxArea;
  Data<float> maxVariation;
  Data<float> minDiversity;
  Data<int> maxEvolution;
  Data<double> areaThreshold;
  Data<double> minMargin;
  Data<int> edgeBlurSize;
};
struct ORBDetector: BaseDetector
{
  ORBDetector(FeatureDetector* c);
  void toggleVisible(bool);

  Data<int> nFeatures;
  Data<float> scaleFactor;
  Data<int> nLevels;
  Data<int> edgeThreshold;
  Data<int> firstLevel;
  Data<int> WTA_K;
  Data<sofa::helper::OptionsGroup> scoreType;
  Data<int> patchSize;
  Data<int> fastThreshold;
};
struct BRISKDetector : BaseDetector
{
  BRISKDetector(FeatureDetector* c);
  void toggleVisible(bool);

  Data<int> threshold;
  Data<int> octaves;
  Data<float> npatternScale;
};
struct KAZEDetector: BaseDetector
{
  KAZEDetector(FeatureDetector* c);
  void toggleVisible(bool);

  Data<bool> extended;
  Data<bool> upright;
  Data<float> threshold;
  Data<int> octaves;
  Data<int> sublevels;
  Data<sofa::helper::OptionsGroup> diffusivity;
};
struct AKAZEDetector: BaseDetector
{
  AKAZEDetector(FeatureDetector* c);
  void toggleVisible(bool);

  Data<sofa::helper::OptionsGroup> descriptorType;
  Data<int> descriptorSize;
  Data<int> descriptorChannels;
  Data<float> threshold;
  Data<int> octaves;
  Data<int> sublevels;
  Data<sofa::helper::OptionsGroup> diffusivity;
};
struct SIFTDetector: BaseDetector
{
  SIFTDetector(FeatureDetector* c);
  void toggleVisible(bool);

  Data<int> nFeatures;
  Data<int> nOctaveLayers;
  Data<double> contrastThreshold;
  Data<double> edgeThreshold;
  Data<double> sigma;
};

struct BRIEFDetector: BaseDetector
{
  BRIEFDetector(FeatureDetector* c);
  void toggleVisible(bool);

  Data<int> bytes;
  Data<bool> use_orientation;
};

}  // namespace sofa
}  // namespace OR
}  // namespace processor

#endif  // SOFA_OR_PROCESSOR_DETECTORS_H
