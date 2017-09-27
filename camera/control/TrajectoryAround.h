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
class TrajectoryAround : public common::ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      TrajectoryAround, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Matrix3 Matrix3;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  typedef typename sofa::defaulttype::Quat Quat;

  SOFAOR_CALLBACK_SYSTEM(TrajectoryAround);

 public:
  SOFA_CLASS(TrajectoryAround, common::ImplicitDataEngine);

  TrajectoryAround()
      : l_cam(initLink("cam", "camera to control")),
        d_center(initData(&d_center, "center",
                          "Point in world coordinates to rotate around")),
        d_theta(initData(&d_theta, "delta_theta",
                         "longitudinal angle to add at each step")),
        d_phi(initData(&d_phi, "delta_phi",
                       "polar (colatitude) angle to add at each step")),
        d_rho(initData(&d_rho, "delta_rho",
                       "distance to add to the sphere's radius at each step")),
        d_thetaInit(
            initData(&d_thetaInit, "theta", "Initial longitudinal angle")),
        d_phiInit(
            initData(&d_phiInit, "phi", "initial polar (colatitude) angle")),
        d_rhoInit(initData(&d_rhoInit, "rho", "initial sphere's radius"))
  {
  }

  ~TrajectoryAround() {}
  void init()
  {
    SOFAOR_ADD_CALLBACK(&d_center, &TrajectoryAround::centerChanged);
    SOFAOR_ADD_CALLBACK(&d_theta, &TrajectoryAround::thetaChanged);
    SOFAOR_ADD_CALLBACK(&d_phi, &TrajectoryAround::phiChanged);
    SOFAOR_ADD_CALLBACK(&d_rho, &TrajectoryAround::rhoChanged);

    addInput(&d_center);
    addInput(&d_theta);
    addInput(&d_phi);
    addInput(&d_rho);

    if (!l_cam.get())
      msg_error(getName() + "::init()") << "Error: No camera link set. ";
    m_rho = d_rho.getValue();
    m_theta = d_theta.getValue();
    m_phi = d_phi.getValue();
    rotate(0, 0, 0, true);
  }

  void rotate(double rho, double theta, double phi, bool init = false)
  {
    Vector3 p = Vector3(0, 0, d_rhoInit.getValue());
    Vector3 sphCoord;

    // convert to spherical coordinates and add rho, theta & phi
    // rho
    sphCoord.x() =
        std::sqrt((p.x() * p.x()) + (p.y() * p.y()) + (p.z() * p.z()));
    // theta
    sphCoord.y() = d_thetaInit.getValue() + std::acos(p.z() / sphCoord.x());
    // phi
    sphCoord.z() = d_phiInit.getValue() + std::atan2(p.y(), p.x());

    if (init)
    {
    }
    else
    {
      sphCoord.x() += m_rho;
      sphCoord.x() = std::abs(sphCoord.x());
      sphCoord.y() += m_theta;
      sphCoord.z() += m_phi;
      m_rho += rho;
      m_theta += theta;
      m_phi += phi;
    }
    // convert back to cartesian coordinates
    p.x() = sphCoord.x() * std::sin(sphCoord.y()) * std::cos(sphCoord.z());
    p.y() = sphCoord.x() * std::sin(sphCoord.y()) * std::sin(sphCoord.z());
    p.z() = sphCoord.x() * std::cos(sphCoord.y());

    Quat q1 = Quat(Vector3(1.0, 0.0, 0.0), M_PI);
    Quat q2 = Quat::fromEuler(0, -sphCoord.y()+ M_PI, 0);
    Quat q3 = Quat::fromEuler(0, 0, -sphCoord.z());
    Quat q4 = Quat(Vector3(0.0, 0.0, 1.0), -M_PI / 2);
    Quat q5 = Quat(Vector3(1.0, 0.0, 0.0), -M_PI / 2);

    Matrix3 R1, R2, R3, R4, R5;
    q1.toMatrix(R1);
    q2.toMatrix(R2);
    q3.toMatrix(R3);
    q4.toMatrix(R4);
    q5.toMatrix(R5);

    p = Vector3(p.x(), -p.z(), p.y());
    p += d_center.getValue();
    Matrix3 R =    R4 * R2 * R3* R5 ;

    l_cam->setPosition(p, false);
    l_cam->setRotationMatrix(R, false);
    l_cam->buildFromKRT();
  }

  void update()
  {
    rotate(d_rho.getValue(), d_theta.getValue(), d_phi.getValue());
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
  sofa::Data<double> d_theta;
  sofa::Data<double> d_phi;
  sofa::Data<double> d_rho;

  Vector3 m_pole;
  sofa::Data<double> d_thetaInit;
  sofa::Data<double> d_phiInit;
  sofa::Data<double> d_rhoInit;

  double m_theta;
  double m_phi;
  double m_rho;

 private:
  void centerChanged(sofa::core::objectmodel::BaseData*) {}
  void thetaChanged(sofa::core::objectmodel::BaseData*)
  {
    //      m_theta = d_theta.getValue();
  }
  void phiChanged(sofa::core::objectmodel::BaseData*)
  {
    //      m_phi = d_phi.getValue();
  }
  void rhoChanged(sofa::core::objectmodel::BaseData*)
  {
    //      m_rho = d_rho.getValue();
  }

  void thetaInitChanged(sofa::core::objectmodel::BaseData*)
  {
    rotate(0, 0, 0, true);
  }
  void phiInitChanged(sofa::core::objectmodel::BaseData*)
  {
    rotate(0, 0, 0, true);
  }
  void rhoInitChanged(sofa::core::objectmodel::BaseData*)
  {
    rotate(0, 0, 0, true);
  }
};

SOFA_DECL_CLASS(TrajectoryAround)

int TrajectoryAroundClass =
    sofa::core::RegisterObject(
        "Component to rotate a camera following any trajectory on a sphere, "
        "around a point. Init sets the correct camera orientation if necessary")
        .add<TrajectoryAround>();

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_ROTATEAROUNDENGINE_H
