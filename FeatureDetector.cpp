#include "FeatureDetector.h"
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(FeatureDetector)

int FeatureDetectorClass =
    core::RegisterObject(
        "debug component to visualize images using OpenCV highgui")
        .add<FeatureDetector>();

FeatureDetector::FeatureDetector()
    : ImageFilter(false),
      d_detectMode(initData(&d_detectMode, "detectorMode",
                            "if true, does not compute the descriptors")),
      d_mask(initData(&d_mask, common::cvMat(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(
          initData(&d_detectorType, "detectorType",
                   "Available feature detection algoritms: FAST, "
                   "MSER, ORB, BRISK, KAZE, AKAZE, SIFT, SURF, BRIEF, DAISY")),
      d_keypoints(
          initData(&d_keypoints, "keypoints", "output array of cvKeypoints", false)),
      d_descriptors(initData(&d_descriptors, "descriptors",
                             "output cvMat of feature descriptors", false, true))
{
  addAlias(&d_keypoints, "keypoints_out");
  addAlias(&d_descriptors, "descriptors_out");
  m_outputImage = false;

  sofa::helper::OptionsGroup* t = d_detectMode.beginEdit();
  t->setNames(3, "DETECT_ONLY", "COMPUTE_ONLY", "DETECT_AND_COMPUTE");
  t->setSelectedItem("DETECT_AND_COMPUTE");
  d_detectMode.endEdit();

  t = d_detectorType.beginEdit();
  t->setNames(DetectorType_COUNT, "FAST", "MSER", "ORB", "BRISK", "KAZE",
              "AKAZE", "SIFT", "SURF", "BRIEF", "DAISY");
  t->setSelectedItem("SIFT");
  d_detectorType.endEdit();

  m_detectors[FAST] = new FASTDetector(this);
  m_detectors[MSER] = new MSERDetector(this);
  m_detectors[ORB] = new ORBDetector(this);
  m_detectors[BRISK] = new BRISKDetector(this);
  m_detectors[KAZE] = new KAZEDetector(this);
  m_detectors[AKAZE] = new AKAZEDetector(this);
  m_detectors[SIFT] = new SIFTDetector(this);
  m_detectors[SURF] = new SURFDetector(this);
  m_detectors[BRIEF] = new BRIEFDetector(this);
  m_detectors[DAISY] = new DAISYDetector(this);
}

FeatureDetector::~FeatureDetector() {}
void FeatureDetector::init()
{
  std::cout << "Detector type: " << d_detectorType.getValue().getSelectedItem()
            << std::endl;
  for (size_t i = 0; i < DetectorType_COUNT; ++i)
  {
    m_detectors[i]->init();
    if (d_detectorType.getValue().getSelectedId() != i)
      m_detectors[i]->toggleVisible(false);
    else
    {
      unregisterAllData();
      m_detectors[i]->registerData(this);
      m_detectors[i]->init();
    }
  }

  switch (d_detectMode.getValue().getSelectedId())
  {
    case DETECT_ONLY:
      d_descriptors.setDisplayed(false);
      addInput(&d_mask);
      delOutput(&d_descriptors);
      delInput(&d_keypoints);
      addOutput(&d_keypoints);
      break;
    case COMPUTE_ONLY:
      d_descriptors.setDisplayed(true);
      delInput(&d_mask);
      delOutput(&d_keypoints);
      bindInputData(&d_keypoints);
      addOutput(&d_descriptors);
      break;
    case DETECT_AND_COMPUTE:
      d_descriptors.setDisplayed(true);
      addInput(&d_mask);
      delInput(&d_keypoints);
      addOutput(&d_keypoints);
      addOutput(&d_descriptors);
      break;
  }
  m_dataTracker.trackData(d_detectMode);
  m_dataTracker.trackData(d_detectorType);

  setDirtyValue();
  ImageFilter::init();
}

void FeatureDetector::update()
{
  std::cout << getName() << std::endl;
  ImageFilter::update();

  switch (d_detectMode.getValue().getSelectedId())
  {
    case DETECT_ONLY:
    {
      sofa::helper::vector<common::cvKeypoint>* vec = d_keypoints.beginEdit();
      vec->clear();
      for (cv::KeyPoint& kp : _v) vec->push_back(common::cvKeypoint(kp));
      d_keypoints.endEdit();
      d_keypoints.setDirtyOutputs();
      break;
    }
    case COMPUTE_ONLY:
    {
      d_descriptors.setValue(_d);
      d_keypoints.cleanDirty();
      d_descriptors.setDirtyOutputs();
      break;
    }
    case DETECT_AND_COMPUTE:
    {
      sofa::helper::vector<common::cvKeypoint>* vec =
          d_keypoints.beginWriteOnly();
      vec->clear();
      for (cv::KeyPoint& kp : _v) vec->push_back(common::cvKeypoint(kp));
      d_keypoints.endEdit();
      d_descriptors.setValue(_d);
      d_keypoints.setDirtyOutputs();
      d_descriptors.setDirtyOutputs();
      break;
    }
  }
  std::cout << "end" << getName() << std::endl;
}

void FeatureDetector::applyFilter(const cv::Mat& in, cv::Mat& out, bool debug)
{
  if (!f_listening.getValue()) return;

  int detect = int(d_detectMode.getValue().getSelectedId());
  if (detect == DETECT_ONLY)
  {
    if (in.empty()) return;
    _v.clear();
    if (!d_keypoints.getValue().empty() && !debug)
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      _v.assign(arr, arr + d_keypoints.getValue().size());
    }
    m_detectors[d_detectorType.getValue().getSelectedId()]->init();
    m_detectors[d_detectorType.getValue().getSelectedId()]->detect(
        in, d_mask.getValue(), _v);
  }
  else if (detect == COMPUTE_ONLY)
  {
    _v.clear();
    if (!d_keypoints.getValue().empty() && !debug)
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      _v.assign(arr, arr + d_keypoints.getValue().size());
    }
    _d = cv::Mat();
    m_detectors[d_detectorType.getValue().getSelectedId()]->init();
    m_detectors[d_detectorType.getValue().getSelectedId()]->compute(in, _v, _d);
  }
  else
  {
    if (in.empty()) return;
    _v.clear();
    if (!d_keypoints.getValue().empty() && !debug)
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      _v.assign(arr, arr + d_keypoints.getValue().size());
    }
    _d = cv::Mat();
    m_detectors[d_detectorType.getValue().getSelectedId()]->init();
    m_detectors[d_detectorType.getValue().getSelectedId()]->detectAndCompute(
        in, d_mask.getValue(), _v, _d);
  }
  if (d_displayDebugWindow.getValue())
  {
    in.copyTo(out);
    cv::drawKeypoints(in, _v, out, cv::Scalar(0, 255, 0), 1);
  }
}

void FeatureDetector::reinit()
{
  if (m_dataTracker.isDirty(d_detectorType))
  {
    for (size_t i = 0; i < DetectorType_COUNT; ++i)
    {
      if (i == d_detectorType.getValue().getSelectedId())
      {
        m_detectors[i]->toggleVisible(true);
        m_detectors[i]->init();
        unregisterAllData();
        m_detectors[i]->registerData(this);
      }
      else
        m_detectors[i]->toggleVisible(false);
    }
  }
  if (m_dataTracker.isDirty(d_detectMode))
  {
    switch (d_detectMode.getValue().getSelectedId())
    {
      case DETECT_ONLY:
        d_descriptors.setDisplayed(false);
        addInput(&d_mask);
        delOutput(&d_descriptors);
        delInput(&d_keypoints);
        addOutput(&d_keypoints);
        break;
      case COMPUTE_ONLY:
        d_descriptors.setDisplayed(true);
        delInput(&d_mask);
        delOutput(&d_keypoints);
        addInput(&d_keypoints);
        addOutput(&d_descriptors);
        break;
      case DETECT_AND_COMPUTE:
        d_descriptors.setDisplayed(true);
        addInput(&d_mask);
        delInput(&d_keypoints);
        addOutput(&d_keypoints);
        addOutput(&d_descriptors);
        break;
    }
  }
  ImageFilter::reinit();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
