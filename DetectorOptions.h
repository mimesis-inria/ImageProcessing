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

struct ComputeOpts
{
  ComputeOpts(FeatureDetector* c);
  void toggleVisible(bool);

  Data<bool> d_useProvidedKeypoints;
  Data<common::cvMat> d_descriptors;
};

struct BaseOpts
{
  virtual ~BaseOpts();

  virtual void toggleVisible(bool) = 0;

  virtual void detect(const common::cvMat&, const common::cvMat&,
                      std::vector<cv::KeyPoint>&);
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&,
                                bool);

 protected:
  cv::Ptr<cv::Feature2D> m_detector;
};
struct FASTOpts : BaseOpts
{
  FASTOpts(FeatureDetector* c);
  void toggleVisible(bool);
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&,
                                bool)
  {
    msg_warning("FASTOpts::detectAndCompute()")
        << "FAST is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
  }

  Data<int> threshold;
  Data<bool> nonmaxsuppression;
  Data<sofa::helper::OptionsGroup> type;
};
struct MSEROpts : BaseOpts
{
  MSEROpts(FeatureDetector* c);
  void toggleVisible(bool);
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&,
                                bool)
  {
    msg_warning("MSEROpts::detectAndCompute()")
        << "MSER is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
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
struct ORBOpts : BaseOpts
{
  ORBOpts(FeatureDetector* c);
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
struct BRISKOpts : BaseOpts
{
  BRISKOpts(FeatureDetector* c);
  void toggleVisible(bool);

  Data<int> threshold;
  Data<int> octaves;
  Data<float> npatternScale;
};
struct KAZEOpts : BaseOpts
{
  KAZEOpts(FeatureDetector* c);
  void toggleVisible(bool);

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
  void toggleVisible(bool);

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
  void toggleVisible(bool);

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
