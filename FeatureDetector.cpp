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
    : d_detect(initData(&d_detect, false, "detect",
                        "if true, does not compute the descriptors", true, true)),
      d_image(
          initData(&d_image, "image", "Image on which to look for features.")),
      d_mask(initData(&d_mask, common::cvMat(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(initData(&d_detectorType, "detectorType",
                              "Available feature detection algoritms: FAST, "
                              "MSER, ORB, BRISK, KAZE, AKAZE, SIFT")),
      d_keypoints(
          initData(&d_keypoints, "keypoints", "output array of cvKeypoints")),
      d_descriptors(initData(&d_descriptors, "descriptors",
                             "output cvMat of feature descriptors", true, true))
{
  f_listening.setValue(true);

  if (d_detect.getValue())
  {
    d_descriptors.setDisplayed(false);
  }

  sofa::helper::OptionsGroup* t = d_detectorType.beginEdit();
  t->setNames(DetectorType_COUNT, "FAST", "MSER", "ORB", "BRISK", "KAZE",
              "AKAZE", "SIFT");
  t->setSelectedItem(5);
  d_detectorType.endEdit();

  m_detectors[FAST] = new FASTDetector(this);
  m_detectors[MSER] = new MSERDetector(this);
  m_detectors[ORB] = new ORBDetector(this);
  m_detectors[BRISK] = new BRISKDetector(this);
  m_detectors[KAZE] = new KAZEDetector(this);
  m_detectors[AKAZE] = new AKAZEDetector(this);
  m_detectors[SIFT] = new SIFTDetector(this);
}

FeatureDetector::~FeatureDetector() {}
void FeatureDetector::init()
{
  std::cout << "Detector type: " << d_detectorType.getValue().getSelectedItem()
            << std::endl;
  for (size_t i = 0; i < DetectorType_COUNT; ++i)
  {
    if (d_detectorType.getValue().getSelectedId() != i)
      m_detectors[i]->toggleVisible(false);
  }

  addInput(&d_image);
  addInput(&d_mask);
  if (!d_detect.getValue())
    addOutput(&d_keypoints);
  else
  {
    addInput(&d_keypoints);
    addOutput(&d_descriptors);
  }
  setDirtyValue();
}
void FeatureDetector::update()
{
  cleanDirty();
  std::vector<cv::KeyPoint> v;
  if (!d_keypoints.getValue().empty())
  {
    const cv::KeyPoint* arr =
        dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

    v.assign(arr, arr + d_keypoints.getValue().size());
  }

  bool detect = d_detect.getValue();
  if (detect)
  {
    m_detectors[d_detectorType.getValue().getSelectedId()]->detect(
        d_image.getValue(), d_mask.getValue(), v);
    sofa::helper::vector<common::cvKeypoint>* vec = d_keypoints.beginEdit();
    vec->clear();
    for (cv::KeyPoint& kp : v) vec->push_back(common::cvKeypoint(kp));
    d_keypoints.endEdit();
  }
  else
  {
    common::cvMat* descr = d_descriptors.beginEdit();
    m_detectors[d_detectorType.getValue().getSelectedId()]->compute(
        d_image.getValue(), v, *descr);
    d_descriptors.endEdit();
  }
}
void FeatureDetector::reinit()
{
  for (size_t i = 0; i < DetectorType_COUNT; ++i)
  {
    if (i == d_detectorType.getValue().getSelectedId())
      m_detectors[i]->toggleVisible(true);
    else
      m_detectors[i]->toggleVisible(false);
  }
}

void FeatureDetector::handleEvent(sofa::core::objectmodel::Event* e)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(e)) this->update();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
