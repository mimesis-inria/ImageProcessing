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
    : d_mask(initData(&d_mask, common::Image(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(initData(&d_detectorType, "type",
                              "Available feature detection algoritms: FAST, "
                              "MSER, ORB, BRISK, KAZE, AKAZE, SIFT"))
{
  f_listening.setValue(true);
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

FeatureDetector::~FeatureDetector() {}
void FeatureDetector::init()
{
  switch (d_detectorType.getValue().getSelectedId())
  {
    case FAST:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
    case MSER:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
    case ORB:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
    case BRISK:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
    case KAZE:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
    case AKAZE:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
    case SIFT:
      m_detector->detect(d_image.getValue(), d_mask.getValue(),
                         d_keypoints.beginEdit());
      break;
  }
}
void FeatureDetector::update() {}
void FeatureDetector::reinit() { update(); }
void FeatureDetector::handleEvent(sofa::core::objectmodel::Event* e)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(e)) this->update();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
