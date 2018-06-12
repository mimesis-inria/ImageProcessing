#ifndef SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
#define SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H

#include "ImageProcessingPlugin.h"
#include "camera/common/CameraSettings.h"

#include <SofaCV/SofaCV.h>
#include <sofa/helper/OptionsGroup.h>

namespace sofacv
{
namespace cam
{
namespace control
{
class SOFA_IMAGEPROCESSING_API CameraTrajectory : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CameraTrajectory, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  enum Plane
  {
    PLANE_XY = 0,
    PLANE_XZ = 1,
    PLANE_YZ = 2,
    PLANE_OTHER = 3
  };

 public:
  SOFA_CLASS(CameraTrajectory, ImplicitDataEngine);

  CameraTrajectory();

  virtual ~CameraTrajectory() override {}
  virtual void init() override;

  virtual void Update() override;

  virtual void handleEvent(sofa::core::objectmodel::Event* e) override;

  CamSettings l_cam;
  sofa::Data<Vector3> d_center;
  sofa::Data<double> d_angle;
  sofa::Data<sofa::helper::OptionsGroup> d_plane;

  double m_radius;

 private:
  void centerChanged() {}
  void angleChanged() {}
  void planeChanged() {}
};

SOFA_DECL_CLASS(CameraTrajectory)

int CameraTrajectoryClass = sofa::core::RegisterObject(
                                "Component to rotate a camera following a "
                                "trajectory on a plane, around a point")
                                .add<CameraTrajectory>();

}  // namespace control
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
