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
    : d_detectOnly(initData(&d_detectOnly, false, "detectOnly",
                            "if true, does not compute the descriptors")),
      d_image(
          initData(&d_image, "image", "Image on which to look for features.")),
      d_mask(initData(&d_mask, common::cvMat(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(initData(&d_detectorType, "detectorType",
                              "Available feature detection algoritms: FAST, "
                              "MSER, ORB, BRISK, KAZE, AKAZE, SIFT")),
      d_keypoints(initData(&d_keypoints, "keypoints",
                           "output array of cvKeypoints", true, true))
{
  f_listening.setValue(true);

  m_compute = new ComputeOpts(this);
  if (d_detectOnly.getValue()) m_compute->toggleVisible(false);

  sofa::helper::OptionsGroup* t = d_detectorType.beginEdit();
  t->setNames(DetectorType_COUNT, "FAST", "MSER", "ORB", "BRISK", "KAZE",
              "AKAZE", "SIFT");
  t->setSelectedItem(5);
  d_detectorType.endEdit();

  std::cout << "detectorType: " << d_detectorType.getValue().getSelectedItem()
            << std::endl;

  m_detectors[FAST] = new FASTOpts(this);
  m_detectors[MSER] = new MSEROpts(this);
  m_detectors[ORB] = new ORBOpts(this);
  m_detectors[BRISK] = new BRISKOpts(this);
  m_detectors[KAZE] = new KAZEOpts(this);
  m_detectors[AKAZE] = new AKAZEOpts(this);
  m_detectors[SIFT] = new SIFTOpts(this);

  for (size_t i = 0; i < DetectorType_COUNT; ++i)
  {
    if (d_detectorType.getValue().getSelectedId() != i)
      m_detectors[i]->toggleVisible(false);
  }
}

FeatureDetector::~FeatureDetector() {}
void FeatureDetector::init()
{
  addInput(&d_image);
  addInput(&d_mask);
  if (m_compute && m_compute->d_useProvidedKeypoints.getValue())
    addInput(&d_keypoints);
  setDirtyValue();
}
void FeatureDetector::update()
{
  std::vector<cv::KeyPoint> v;
  if (!d_keypoints.getValue().empty())
  {
    const cv::KeyPoint* arr =
        dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

    v.assign(arr, arr + d_keypoints.getValue().size());
  }

  bool detectOnly = d_detectOnly.getValue();
  bool useKPts =
      (m_compute) ? (m_compute->d_useProvidedKeypoints.getValue()) : (false);
  if (detectOnly)
    m_detectors[d_detectorType.getValue().getSelectedId()]->detect(
        d_image.getValue(), d_mask.getValue(), v);
  else
  {
    common::cvMat* descr = m_compute->d_descriptors.beginEdit();
    m_detectors[d_detectorType.getValue().getSelectedId()]->detectAndCompute(
        d_image.getValue(), d_mask.getValue(), v, *descr, useKPts);
    m_compute->d_descriptors.endEdit();
  }
  cleanDirty();

  sofa::helper::vector<common::cvKeypoint>* vec = d_keypoints.beginEdit();
  vec->clear();
  for (cv::KeyPoint& kp : v) vec->push_back(common::cvKeypoint(kp));
  d_keypoints.endEdit();
}
void FeatureDetector::reinit()
{
  if (d_detectOnly.getValue())
    m_compute->toggleVisible(false);
  else
    m_compute->toggleVisible(true);

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
