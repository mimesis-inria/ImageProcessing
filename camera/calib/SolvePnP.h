#ifndef SOFACV_CAM_CALIB_SOLVEPNP_H
#define SOFACV_CAM_CALIB_SOLVEPNP_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include "camera/common/CameraSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
namespace calib
{
/**
 * @brief The SolvePnP class
 *
 * Estimates the pose of a camera relative to a 3D object by using a set of
 * points on the object, and their 2D correspondances in the camera's image
 *
 * (see SolvePnP in http://docs.opencv.org/3.2.0/d9/d0c/group__calib3d.html
 * for details)
 */
class SOFA_IMAGEPROCESSING_API SolvePnP : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      SolvePnP, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(SolvePnP, ImplicitDataEngine);

  SolvePnP();

  virtual ~SolvePnP() override {}
  void init() override;

  virtual void Update() override;

  CamSettings l_cam;  ///< Camera settings to update

  // INPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector2> >
      d_imagePoints;  ///< [INPUT] 2D points in the image
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector3> >
      d_objectPoints;  ///< [INPUT] 3D points on the object

  // OPTIONAL INPUT
  sofa::Data<sofa::defaulttype::Vec2i>
      d_imgSize;  ///< [INPUT] image size to estimate K
  sofa::Data<sofa::defaulttype::Matrix3> d_K;  ///< [INPUT] Intrinsic Guess
  sofa::Data<sofa::helper::vector<double> >
      d_distCoefs;             ///< [INPUT] Distortion coefficients guess
  sofa::Data<int> d_pnpFlags;  ///< OpenCV's PNP flags
};

}  // namespace calib
}  // namespace cam
}  // namespace sofacv

#endif  // SOFACV_CAM_CALIB_SOLVEPNP_H
