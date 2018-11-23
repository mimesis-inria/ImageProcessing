#ifndef SOFACV_CAM_CALIB_CALIBRATECAMERA_H
#define SOFACV_CAM_CALIB_CALIBRATECAMERA_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include "camera/common/CameraSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
namespace calib
{
class SOFA_IMAGEPROCESSING_API CalibrateCamera : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CalibrateCamera, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(CalibrateCamera, ImplicitDataEngine);

  CalibrateCamera();

  ~CalibrateCamera() {}
  void init() override;

  virtual void doUpdate() override;
  void calibrate();

  CamSettings l_cam;

  // INPUTS
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector2> > >
      d_imagePoints;
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector3> > >
      d_objectPoints;
  sofa::Data<sofa::defaulttype::Vec2i> d_imgSize;

  // OPTIONAL INPUTS
  sofa::Data<int> d_calibFlags;
  sofa::Data<sofa::defaulttype::Matrix3> d_K;
  sofa::Data<sofa::helper::vector<double> > d_distCoefs;

  // OUTPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Mat3x4d> > d_Rts;

  sofa::Data<bool> d_preserveExtrinsics;

 private:
  sofa::defaulttype::Matrix3 m_K;
  sofa::helper::vector<double> m_distCoefs;
};

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CALIB_CALIBRATECAMERA_H
