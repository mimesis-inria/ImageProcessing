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
struct BaseDetector
{
  virtual ~BaseDetector();

  virtual void toggleVisible(bool) = 0;

  virtual void init() = 0;

  virtual void detect(const common::cvMat&, const common::cvMat&,
                      std::vector<cv::KeyPoint>&);
  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&);
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&);

 protected:
  cv::Ptr<cv::Feature2D> m_detector;
};
struct FASTDetector : BaseDetector
{
  FASTDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_warning("FASTDetector::compute()")
        << "FAST is detectOnly. descriptors won't be computed.";
  }
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&)
  {
    msg_warning("FASTDetector::detectAndCompute()")
        << "FAST is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
  }

  Data<int> threshold;
  Data<bool> nonmaxsuppression;
  Data<sofa::helper::OptionsGroup> type;
};
struct MSERDetector : BaseDetector
{
  MSERDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_warning("MSERDetector::detectAndCompute()")
        << "MSER is detectOnly. descriptors won't be computed.";
  }
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&)
  {
    msg_warning("MSERDetector::detectAndCompute()")
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
struct ORBDetector : BaseDetector
{
  ORBDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

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
  BRISKDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

  Data<int> threshold;
  Data<int> octaves;
  Data<float> npatternScale;
};
struct KAZEDetector : BaseDetector
{
  KAZEDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

  Data<bool> extended;
  Data<bool> upright;
  Data<float> threshold;
  Data<int> octaves;
  Data<int> sublevels;
  Data<sofa::helper::OptionsGroup> diffusivity;
};
struct AKAZEDetector : BaseDetector
{
  AKAZEDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

  Data<sofa::helper::OptionsGroup> descriptorType;
  Data<int> descriptorSize;
  Data<int> descriptorChannels;
  Data<float> threshold;
  Data<int> octaves;
  Data<int> sublevels;
  Data<sofa::helper::OptionsGroup> diffusivity;
};
struct SIFTDetector : BaseDetector
{
  SIFTDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

  Data<int> nFeatures;
  Data<int> nOctaveLayers;
  Data<double> contrastThreshold;
  Data<double> edgeThreshold;
  Data<double> sigma;
};

struct SURFDetector : BaseDetector
{
  SURFDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

  Data<double> threshold;
  Data<int> nOctaves;
  Data<int> nOctaveLayers;
  Data<bool> extended;
  Data<bool> upright;
};

struct BRIEFDetector : BaseDetector
{
  BRIEFDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  void detect(const common::cvMat&, const common::cvMat&,
              std::vector<cv::KeyPoint>&)
  {
    msg_warning("BRIEFDetector::detect()")
        << "BRIEF is computeOnly. keypoints must be provided.";
  }
  virtual void detectAndCompute(const common::cvMat&,
                                const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&)
  {
    msg_warning("BRIEFDetector::detectAndCompute()")
        << "BRIEF is computeOnly. Please provide keypoints and set "
           "DetectMode to COMPUTE_ONLY";
  }

  Data<int> bytes;
  Data<bool> use_orientation;
};

struct DAISYDetector : BaseDetector
{
  DAISYDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  void detect(const common::cvMat&, const common::cvMat&,
              std::vector<cv::KeyPoint>&)
  {
    msg_warning("DAISYDetector::detect()")
        << "DAISY is computeOnly. keypoints must be provided.";
  }
  virtual void detectAndCompute(const common::cvMat&,
                                const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&)
  {
    msg_warning("DAISYDetector::detectAndCompute()")
        << "DAISY is computeOnly. Please provide keypoints and set "
           "DetectMode to COMPUTE_ONLY";
  }

  Data<float> radius;
  Data<int> q_radius;
  Data<int> q_theta;
  Data<int> q_hist;
  Data<sofa::helper::OptionsGroup> norm;
  Data<common::cvMat> H;
  Data<bool> interpolation;
  Data<bool> use_orientation;
};

}  // namespace sofa
}  // namespace OR
}  // namespace processor

#endif  // SOFA_OR_PROCESSOR_DETECTORS_H
