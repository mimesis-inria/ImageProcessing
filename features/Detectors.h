#ifndef SOFA_OR_PROCESSOR_DETECTORS_H
#define SOFA_OR_PROCESSOR_DETECTORS_H

#include <SofaCV/SofaCV.h>
#include "common/ImageFilter.h"

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vector.h>

#include <opencv2/xfeatures2d.hpp>

namespace sofacv
{
namespace features
{
struct BaseDetector
{
  BaseDetector(sofa::core::DataEngine* c)
  {
    m_obj = c;
  }

  virtual ~BaseDetector();

  virtual void enable(bool) = 0;

  virtual void init() = 0;

  virtual void registerData(common::ImageFilter* parent) = 0;

  virtual void detect(const cvMat&, const cvMat&,
                      std::vector<cv::KeyPoint>&);
  virtual void compute(const cvMat&, std::vector<cv::KeyPoint>&,
                       cvMat&);
  virtual void detectAndCompute(const cvMat&, const cvMat&,
                                std::vector<cv::KeyPoint>&, cvMat&);

 protected:
  cv::Ptr<cv::Feature2D> m_detector;
  sofa::core::DataEngine* m_obj;
};

struct SimpleBlobDetector : BaseDetector
{
  SimpleBlobDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();

  virtual void registerData(common::ImageFilter* parent);

  virtual void detect(const cvMat& img, const cvMat& mask,
                      std::vector<cv::KeyPoint>& keypoints);

  virtual void compute(const cvMat&, std::vector<cv::KeyPoint>&,
                       cvMat&);
  virtual void detectAndCompute(const cvMat& img,
                                const cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts,
                                cvMat&);

  sofa::Data<int> thresholdStep;
  sofa::Data<int> minThreshold;
  sofa::Data<int> maxThreshold;
  sofa::Data<int> minDistBetweenBlobs;
  sofa::Data<bool> filterByColor;
  sofa::Data<int> blobColor;
  sofa::Data<bool> filterByArea;
  sofa::Data<int> minArea;
  sofa::Data<bool> filterByCircularity;
  sofa::Data<double> minCircularity;
  sofa::Data<bool> filterByConvexity;
  sofa::Data<double> minConvexity;
  sofa::Data<bool> filterByInertia;
  sofa::Data<double> minInertiaRatio;
};

struct ShiTomasiDetector : BaseDetector
{
  ShiTomasiDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter* parent);

  virtual void detect(const cvMat&, const cvMat&,
                      std::vector<cv::KeyPoint>&);

  virtual void compute(const cvMat&, std::vector<cv::KeyPoint>&,
                       cvMat&);
  virtual void detectAndCompute(const cvMat& img,
                                const cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts,
                                cvMat&);

  sofa::Data<int> maxCorners;
  sofa::Data<double> qualityLevel;
  sofa::Data<int> minDistance;
  sofa::Data<int> blockSize;
};

struct FASTDetector : BaseDetector
{
  FASTDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter* parent);
  virtual void compute(const cvMat&, std::vector<cv::KeyPoint>&,
                       cvMat&);
  virtual void detectAndCompute(const cvMat& img,
                                const cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts,
                                cvMat&);

  sofa::Data<int> threshold;
  sofa::Data<bool> nonmaxsuppression;
  sofa::Data<sofa::helper::OptionsGroup> type;
};
struct MSERDetector : BaseDetector
{
  MSERDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter* parent);

  virtual void compute(const cvMat&, std::vector<cv::KeyPoint>&,
                       cvMat&);
  virtual void detectAndCompute(const cvMat& img,
                                const cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts,
                                cvMat&);

  sofa::Data<int> delta;
  sofa::Data<int> minArea;
  sofa::Data<int> maxArea;
  sofa::Data<float> maxVariation;
  sofa::Data<float> minDiversity;
  sofa::Data<int> maxEvolution;
  sofa::Data<double> areaThreshold;
  sofa::Data<double> minMargin;
  sofa::Data<int> edgeBlurSize;
};
struct ORBDetector : BaseDetector
{
  ORBDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();

  virtual void registerData(common::ImageFilter*);

  sofa::Data<int> nFeatures;
  sofa::Data<float> scaleFactor;
  sofa::Data<int> nLevels;
  sofa::Data<int> edgeThreshold;
  sofa::Data<int> firstLevel;
  sofa::Data<int> WTA_K;
  sofa::Data<sofa::helper::OptionsGroup> scoreType;
  sofa::Data<int> patchSize;
  sofa::Data<int> fastThreshold;
};
struct BRISKDetector : BaseDetector
{
  BRISKDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  sofa::Data<int> threshold;
  sofa::Data<int> octaves;
  sofa::Data<float> npatternScale;
};
struct KAZEDetector : BaseDetector
{
  KAZEDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  sofa::Data<bool> extended;
  sofa::Data<bool> upright;
  sofa::Data<float> threshold;
  sofa::Data<int> octaves;
  sofa::Data<int> sublevels;
  sofa::Data<sofa::helper::OptionsGroup> diffusivity;
};
struct AKAZEDetector : BaseDetector
{
  AKAZEDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  sofa::Data<sofa::helper::OptionsGroup> descriptorType;
  sofa::Data<int> descriptorSize;
  sofa::Data<int> descriptorChannels;
  sofa::Data<float> threshold;
  sofa::Data<int> octaves;
  sofa::Data<int> sublevels;
  sofa::Data<sofa::helper::OptionsGroup> diffusivity;
};

#ifdef SOFACV_OPENCV_CONTRIB_ENABLED

struct BRIEFDetector : BaseDetector
{
  BRIEFDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  void detect(const cvMat&, const cvMat&,
              std::vector<cv::KeyPoint>&);
  virtual void detectAndCompute(const cvMat&, const cvMat&,
                                std::vector<cv::KeyPoint>&, cvMat&);

  sofa::Data<int> bytes;
  sofa::Data<bool> use_orientation;
};

struct SIFTDetector : BaseDetector
{
  SIFTDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  sofa::Data<int> nFeatures;
  sofa::Data<int> nOctaveLayers;
  sofa::Data<double> contrastThreshold;
  sofa::Data<double> edgeThreshold;
  sofa::Data<double> sigma;
};

struct SURFDetector : BaseDetector
{
  SURFDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  sofa::Data<double> threshold;
  sofa::Data<int> nOctaves;
  sofa::Data<int> nOctaveLayers;
  sofa::Data<bool> extended;
  sofa::Data<bool> upright;
};

struct DAISYDetector : BaseDetector
{
  DAISYDetector(sofa::core::DataEngine* c);
  void enable(bool);
  void init();
  virtual void registerData(common::ImageFilter*);

  void detect(const cvMat&, const cvMat&,
              std::vector<cv::KeyPoint>&);
  virtual void detectAndCompute(const cvMat&, const cvMat&,
                                std::vector<cv::KeyPoint>&, cvMat&);

  sofa::Data<float> radius;
  sofa::Data<int> q_radius;
  sofa::Data<int> q_theta;
  sofa::Data<int> q_hist;
  sofa::Data<sofa::helper::OptionsGroup> norm;
  sofa::Data<cvMat> H;
  sofa::Data<bool> interpolation;
  sofa::Data<bool> use_orientation;
};

#endif  // SOFACV_OPENCV_CONTRIB_ENABLED

}  // namespace features
}  // namespace sofacv

#endif  // SOFACV_FEATURES_DETECTORS_H
