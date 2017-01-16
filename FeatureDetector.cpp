#include "FeatureDetector.h"

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
    : d_detectMode(initData(&d_detectMode, "detectorMode",
                            "if true, does not compute the descriptors")),
      d_image(
          initData(&d_image, "image", "Image on which to look for features.")),
      d_mask(initData(&d_mask, common::cvMat(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(
          initData(&d_detectorType, "detectorType",
                   "Available feature detection algoritms: FAST, "
                   "MSER, ORB, BRISK, KAZE, AKAZE, SIFT, SURF, BRIEF, DAISY")),
      d_keypoints(
          initData(&d_keypoints, "keypoints", "output array of cvKeypoints")),
      d_descriptors(initData(&d_descriptors, "descriptors",
                             "output cvMat of feature descriptors", true, true))
{
  f_listening.setValue(true);

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
      m_detectors[i]->init();
  }

  addInput(&d_image);

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
  setDirtyValue();
}
void FeatureDetector::update()
{
  if (!f_listening.getValue()) return;

  updateAllInputsIfDirty();
  cleanDirty();
  int detect = int(d_detectMode.getValue().getSelectedId());
  if (detect == DETECT_ONLY)
  {
    std::cout << getName() << std::endl;
    std::vector<cv::KeyPoint> v;
    if (!d_keypoints.getValue().empty())
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      v.assign(arr, arr + d_keypoints.getValue().size());
    }

    m_detectors[d_detectorType.getValue().getSelectedId()]->detect(
        d_image.getValue(), d_mask.getValue(), v);
    sofa::helper::vector<common::cvKeypoint>* vec = d_keypoints.beginEdit();
    vec->clear();
    for (cv::KeyPoint& kp : v) vec->push_back(common::cvKeypoint(kp));
    d_keypoints.endEdit();
    d_image.cleanDirty();
    d_keypoints.setDirtyOutputs();
  }
  else if (detect == COMPUTE_ONLY)
  {
    std::cout << getName() << std::endl;
    std::vector<cv::KeyPoint> v;
    if (!d_keypoints.getValue().empty())
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      v.assign(arr, arr + d_keypoints.getValue().size());
    }

    common::cvMat* descr = d_descriptors.beginEdit();
    m_detectors[d_detectorType.getValue().getSelectedId()]->compute(
        d_image.getValue(), v, *descr);
    d_descriptors.endEdit();
    d_keypoints.cleanDirty();
    d_descriptors.setDirtyOutputs();
  }
  else
  {
    std::cout << getName() << std::endl;
    std::vector<cv::KeyPoint> v;
    if (!d_keypoints.getValue().empty())
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      v.assign(arr, arr + d_keypoints.getValue().size());
    }

    common::cvMat* descr = d_descriptors.beginEdit();
    m_detectors[d_detectorType.getValue().getSelectedId()]->detectAndCompute(
        d_image.getValue(), d_mask.getValue(), v, *descr);
    d_descriptors.endEdit();
    sofa::helper::vector<common::cvKeypoint>* vec =
        d_keypoints.beginWriteOnly();
    vec->clear();
    for (cv::KeyPoint& kp : v) vec->push_back(common::cvKeypoint(kp));
    d_keypoints.endEdit();
    d_image.cleanDirty();
    d_descriptors.setDirtyOutputs();
  }
}
void FeatureDetector::reinit()
{
  for (size_t i = 0; i < DetectorType_COUNT; ++i)
  {
    if (i == d_detectorType.getValue().getSelectedId())
    {
      m_detectors[i]->toggleVisible(true);
      m_detectors[i]->init();
    }
    else
      m_detectors[i]->toggleVisible(false);
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

void FeatureDetector::handleEvent(sofa::core::objectmodel::Event* e)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(e)) this->update();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
