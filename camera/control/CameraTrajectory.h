/******************************************************************************
*       SOFAOR, SOFA plugin for the Operating Room, development version       *
*                        (c) 2017 INRIA, MIMESIS Team                         *
*                                                                             *
* This program is a free software; you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 1.0 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Bruno Marques and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact-mimesis@inria.fr                               *
******************************************************************************/

#ifndef SOFA_OR_PROCESSOR_ROTATEAROUNDENGINE_H
#define SOFA_OR_PROCESSOR_ROTATEAROUNDENGINE_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <sofa/helper/OptionsGroup.h>

namespace sofaor
{
namespace processor
{
class CameraTrajectory : public common::ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CameraTrajectory, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  SOFAOR_CALLBACK_SYSTEM(CameraTrajectory);
  enum Plane
  {
    PLANE_XY = 0,
    PLANE_XZ = 1,
    PLANE_YZ = 2,
    PLANE_OTHER = 3
  };

 public:
  SOFA_CLASS(CameraTrajectory, common::ImplicitDataEngine);

  CameraTrajectory()
      : l_cam(initLink("cam", "camera to control")),
        d_center(
            initData(&d_center, "center", "Point in space to rotate around")),
        d_angle(initData(&d_angle, "angle", "Rotation angle in degrees")),
        d_plane(initData(&d_plane, "plane", "Circular trajectory plane"))
  {
    sofa::helper::OptionsGroup plane(3, "XY", "XZ", "YZ");
    plane.setSelectedItem(0);
    d_plane.setValue(plane);
  }

  ~CameraTrajectory() {}
  void init()
  {
    SOFAOR_ADD_CALLBACK(&d_center, &CameraTrajectory::centerChanged);
    SOFAOR_ADD_CALLBACK(&d_angle, &CameraTrajectory::angleChanged);
    SOFAOR_ADD_CALLBACK(&d_plane, &CameraTrajectory::planeChanged);

    addInput(&d_center);
    addInput(&d_angle);
    addInput(&d_plane);

    if (!l_cam.get())
      msg_error(getName() + "::init()") << "Error: No camera link set. ";
  }

  void update()
  {
    Vector3 p = l_cam->getPosition();
    Vector3 c = d_center.getValue();

    m_radius = std::sqrt((p.x() - c.x()) * (p.x() - c.x()) +
                         (p.y() - c.y()) * (p.y() - c.y()) +
                         (p.z() - c.z()) * (p.z() - c.z()));

    const Vector3& center = d_center.getValue();
    const double& radius = m_radius;
    const double& angle = d_angle.getValue() * M_PI / 180;

    double cosvalue = cos(angle) * radius;
    double sinvalue = sin(angle) * radius;

    p = center;
    const int plane = d_plane.getValue().getSelectedId();
    Vector3 up;
    switch (plane)
    {
      case PLANE_XY:
        p[0] += cosvalue;
        p[1] += sinvalue;
        up = Vector3(0, 0, -1);
        break;
      case PLANE_XZ:
        p[0] += cosvalue;
        p[2] += sinvalue;
        up = Vector3(0, -1, 0);
        break;
      case PLANE_YZ:
        p[1] += cosvalue;
        p[2] += sinvalue;
        up = Vector3(-1, 0, 0);
        break;
      case PLANE_OTHER:
      default:
        break;
    }

    l_cam->setPosition(p, false);
    l_cam->d_lookAt.setValue(center);
    l_cam->d_upVector.setValue(up);
    l_cam->buildFromIntrinsicCamPosLookAtAndUpVector();
    d_angle.setValue(d_angle.getValue() + 1);
  }

  virtual void handleEvent(sofa::core::objectmodel::Event* e)
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
      cleanInputs();
      update();
    }
  }

  CamSettings l_cam;
  sofa::Data<Vector3> d_center;
  sofa::Data<double> d_angle;
  sofa::Data<sofa::helper::OptionsGroup> d_plane;

  double m_radius;

 private:
  void centerChanged(sofa::core::objectmodel::BaseData*) {}
  void angleChanged(sofa::core::objectmodel::BaseData*) {}
  void planeChanged(sofa::core::objectmodel::BaseData*) {}
};

SOFA_DECL_CLASS(CameraTrajectory)

int CameraTrajectoryClass = sofa::core::RegisterObject(
                                "Component to rotate a camera following a "
                                "trajectory on a plane, around a point")
                                .add<CameraTrajectory>();

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_ROTATEAROUNDENGINE_H
