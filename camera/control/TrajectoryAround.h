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

class SOFA_IMAGEPROCESSING_API TrajectoryAround : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      TrajectoryAround, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Matrix3 Matrix3;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  typedef typename sofa::defaulttype::Quat Quat;

 public:
  SOFA_CLASS(TrajectoryAround, ImplicitDataEngine);

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

  virtual ~TrajectoryAround() override{}
  void init() override
  {
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
  Quat q2 = Quat::fromEuler(0, -sphCoord.y() + M_PI, 0);
  Quat q3 = Quat::fromEuler(0, 0, -sphCoord.z());
  Quat q4 = Quat(Vector3(0.0, 0.0, 1.0), -M_PI / 2);
  Quat q5 = Quat(Vector3(0.0, 1.0, 0.0), M_PI / 2);
  Quat q6 = Quat(Vector3(1.0, 0.0, 0.0), -M_PI / 2);
  Quat q7 = Quat(Vector3(0.0, 0.0, 1.0), -M_PI / 2);


  Matrix3 R1, R2, R3, R4, R5, R6, R7;
  q1.toMatrix(R1);
  q2.toMatrix(R2);
  q3.toMatrix(R3);
  q4.toMatrix(R4);
  q5.toMatrix(R5);
  q6.toMatrix(R6);
  q7.toMatrix(R7);

  p = Vector3(-p.z(), -p.x(), p.y());
  p += d_center.getValue();
  Matrix3 R = R7*R4 * R2 * R3 * R5 * R6;

  l_cam->setPosition(p, false);
  l_cam->setRotationMatrix(R, false);
  l_cam->buildFromKRT();
  }

  void Update() override
  {
    if (m_dataTracker.isDirty(d_center))
      centerChanged();
    if (m_dataTracker.isDirty(d_theta))
      thetaChanged();
    if (m_dataTracker.isDirty(d_phi))
      phiChanged();
    if (m_dataTracker.isDirty(d_rho))
      rhoChanged();
    rotate(d_rho.getValue(), d_theta.getValue(), d_phi.getValue());
  }

  virtual void handleEvent(sofa::core::objectmodel::Event* e) override
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
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
  void centerChanged() {}
  void thetaChanged()
  {
    //      m_theta = d_theta.getValue();
  }
  void phiChanged()
  {
    //      m_phi = d_phi.getValue();
  }
  void rhoChanged()
  {
    //      m_rho = d_rho.getValue();
  }

  void thetaInitChanged()
  {
    rotate(0, 0, 0, true);
  }
  void phiInitChanged()
  {
    rotate(0, 0, 0, true);
  }
  void rhoInitChanged()
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

}  // namespace control
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
