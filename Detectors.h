#ifndef SOFA_OR_PROCESSOR_DETECTORS_H
#define SOFA_OR_PROCESSOR_DETECTORS_H

#include <SofaORCommon/cvMat.h>
#include "ImageFilter.h"

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

  virtual void registerData(ImageFilter* parent) = 0;

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
  virtual void registerData(ImageFilter* parent)
  {
      parent->registerData(&threshold, 0, 255, 1);
      parent->registerData(&nonmaxsuppression);
      parent->registerData(&type, 0, 3, 1);
  }
  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_error("FASTDetector::compute()")
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
  virtual void registerData(ImageFilter* parent)
  {
      parent->registerData(&delta, 0, 10, 1);
      parent->registerData(&minArea, 0, 255, 1);
      parent->registerData(&maxArea, 10000, 20000, 200);

      parent->registerData(&maxVariation, 0.0f, 1.0f, 0.01f);
      parent->registerData(&minDiversity, 0.0f, 1.0f, 0.01f);
      parent->registerData(&maxEvolution, 0, 400, 1);
      parent->registerData(&areaThreshold, 1.0, 2.0, 0.01);
      parent->registerData(&minMargin, 0.0, 0.1, 0.0001);
      parent->registerData(&edgeBlurSize, 0, 10, 1);
  }

  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_error("MSERDetector::detectAndCompute()")
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

  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }

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
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }


  Data<int> threshold;
  Data<int> octaves;
  Data<float> npatternScale;
};
struct KAZEDetector : BaseDetector
{
  KAZEDetector(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }


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
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }


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
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }


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
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }


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
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }

  void detect(const common::cvMat&, const common::cvMat&,
              std::vector<cv::KeyPoint>&)
  {
    msg_error("BRIEFDetector::detect()")
        << "BRIEF is computeOnly. keypoints must be provided.";
  }
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&)
  {
    msg_error("BRIEFDetector::detectAndCompute()")
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
  virtual void registerData(ImageFilter* )
  {
      // TODO: find optimal range of values
  }

  void detect(const common::cvMat&, const common::cvMat&,
              std::vector<cv::KeyPoint>&)
  {
    msg_error("DAISYDetector::detect()")
        << "DAISY is computeOnly. keypoints must be provided.";
  }
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&)
  {
    msg_error("DAISYDetector::detectAndCompute()")
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
