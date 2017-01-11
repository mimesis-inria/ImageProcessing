#ifndef SOFA_OR_PROCESSOR_DETECTOROPTIONS_H
#define SOFA_OR_PROCESSOR_DETECTOROPTIONS_H

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

struct BaseOpts
{
  virtual ~BaseOpts();
  void detect(const common::cvMat&, const common::cvMat&,
                      std::vector<cv::KeyPoint>&);
  void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&,
                                common::cvMat&, bool);

 protected:
  cv::Ptr<cv::Feature2D> m_detector;
};
struct FASTOpts : BaseOpts
{
  FASTOpts(FeatureDetector* c);

  Data<int> threshold;
  Data<bool> nonmaxsuppression;
  Data<sofa::helper::OptionsGroup> type;
};
struct MSEROpts : BaseOpts
{
  MSEROpts(FeatureDetector* c);

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
struct ORBOpts : BaseOpts
{
  ORBOpts(FeatureDetector* c);

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
struct BRISKOpts : BaseOpts
{
  BRISKOpts(FeatureDetector* c);

  Data<int> threshold;
  Data<int> octaves;
  Data<float> npatternScale;
};
struct KAZEOpts : BaseOpts
{
  KAZEOpts(FeatureDetector* c);

  Data<bool> extended;
  Data<bool> upright;
  Data<float> threshold;
  Data<int> octaves;
  Data<int> sublevels;
  Data<sofa::helper::OptionsGroup> diffusivity;
};
struct AKAZEOpts : BaseOpts
{
  AKAZEOpts(FeatureDetector* c);

  Data<sofa::helper::OptionsGroup> descriptorType;
  Data<int> descriptorSize;
  Data<int> descriptorChannels;
  Data<float> threshold;
  Data<int> octaves;
  Data<int> sublevels;
  Data<sofa::helper::OptionsGroup> diffusivity;
};
struct SIFTOpts : BaseOpts
{
  SIFTOpts(FeatureDetector* c);

  Data<int> nFeatures;
  Data<int> nOctaveLayers;
  Data<double> contrastThreshold;
  Data<double> edgeThreshold;
  Data<double> sigma;
};

}  // namespace sofa
}  // namespace OR
}  // namespace processor

#endif  // SOFA_OR_PROCESSOR_DETECTOROPTIONS_H
