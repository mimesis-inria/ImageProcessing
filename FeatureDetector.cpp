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
      d_image(initData(&d_image, common::cvMat(), "image",
                       "Image on which to look for features.")),
      d_mask(initData(&d_mask, common::cvMat(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(initData(&d_detectorType, "type",
                              "Available feature detection algoritms: FAST, "
                              "MSER, ORB, BRISK, KAZE, AKAZE, SIFT")),
      d_keypoints(initData(&d_keypoints, "keypoints",
                           "output array of cvKeypoints", true, true))
{
  f_listening.setValue(true);

  if (!d_detectOnly.getValue())
  {
    d_useProvidedKeypoints = new Data<bool>(initData(
        d_useProvidedKeypoints, false, "useProvidedKeypoints",
        "whether or not to perform a feature detection before description"));
    d_descriptors = new Data<common::cvMat>(
        initData(d_descriptors, "descriptors",
                 "output cvMat of feature descriptors", true, true));
  }

  sofa::helper::OptionsGroup* t = d_detectorType.beginEdit();
  t->setNames(7, "FAST", "MSER", "ORB", "BRISK", "KAZE", "AKAZE", "SIFT");
  t->setSelectedItem(5);
  d_detectorType.endEdit();
  switch (d_detectorType.getValue().getSelectedId())
  {
    case FAST:
      m_detector = new FASTOpts(this);
      break;
    case MSER:
      m_detector = new MSEROpts(this);
      break;
    case ORB:
      m_detector = new ORBOpts(this);
      break;
    case BRISK:
      m_detector = new BRISKOpts(this);
      break;
    case KAZE:
      m_detector = new KAZEOpts(this);
      break;
    case AKAZE:
      m_detector = new AKAZEOpts(this);
      break;
    case SIFT:
      m_detector = new SIFTOpts(this);
      break;
  }
}

FeatureDetector::~FeatureDetector()
{
  delete d_useProvidedKeypoints;
  delete d_descriptors;
}
void FeatureDetector::init()
{
  addInput(&d_image);
  addInput(&d_mask);
  if (d_useProvidedKeypoints->getValue()) addInput(&d_keypoints);
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
  bool useKPts = d_useProvidedKeypoints->getValue();
  if (detectOnly)
    m_detector->detect(d_image.getValue(), d_mask.getValue(), v);
  else
  {
    common::cvMat* descr = d_descriptors->beginEdit();
    m_detector->detectAndCompute(d_image.getValue(), d_mask.getValue(), v,
                                 *descr, useKPts);
    d_descriptors->endEdit();
  }
  cleanDirty();

  sofa::helper::vector<common::cvKeypoint>* vec = d_keypoints.beginEdit();
  vec->clear();
  for (cv::KeyPoint& kp : v) vec->push_back(common::cvKeypoint(kp));
  d_keypoints.endEdit();
}
void FeatureDetector::reinit() { update(); }
void FeatureDetector::handleEvent(sofa::core::objectmodel::Event* e)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(e)) this->update();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
