#ifndef SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
#define SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H

#include "ImageProcessingPlugin.h"
#include "camera/common/CameraSettings.h"

#include <SofaCV/SofaCV.h>
#include <sofa/helper/OptionsGroup.h>
#include <fstream>

namespace sofacv
{
namespace cam
{
namespace control
{
class SOFA_IMAGEPROCESSING_API CameraController : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CameraController, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Matrix3 Matrix3;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  typedef typename sofa::defaulttype::Quat Quat;

 public:
  SOFA_CLASS(CameraController, ImplicitDataEngine);

  CameraController()
      : l_cam(initLink("cam", "camera to control")),
        d_tx(initData(
            &d_tx, "tx",
            "optical center position along x in the world reference frame")),
        d_ty(initData(
            &d_ty, "ty",
            "optical center position along y in the world reference frame")),
        d_tz(initData(
            &d_tz, "tz",
            "optical center position along z in the world reference frame")),
        d_R00(initData(&d_R00, "R00", "Rotation matrix 00")),
        d_R01(initData(&d_R01, "R01", "Rotation matrix 01")),
        d_R02(initData(&d_R02, "R02", "Rotation matrix 02")),
        d_R10(initData(&d_R10, "R10", "Rotation matrix 10")),
        d_R11(initData(&d_R11, "R11", "Rotation matrix 11")),
        d_R12(initData(&d_R12, "R12", "Rotation matrix 12")),
        d_R20(initData(&d_R20, "R20", "Rotation matrix 20")),
        d_R21(initData(&d_R21, "R21", "Rotation matrix 21")),
        d_R22(initData(&d_R22, "R22", "Rotation matrix 22")),
        d_imgS(initData(&d_imgS, "d_img", "serialized image"))
  {
  }

  ~CameraController() {}
  void init()
  {
    addInput(&d_tx);
    addInput(&d_ty);
    addInput(&d_tz);

    addInput(&d_R00);
    addInput(&d_R01);
    addInput(&d_R02);

    addInput(&d_R10);
    addInput(&d_R11);
    addInput(&d_R12);

    addInput(&d_R20);
    addInput(&d_R21);
    addInput(&d_R22);

    if (!l_cam.get())
      msg_error(getName() + "::init()") << "Error: No camera link set. ";

    update();
  }

  void doUpdate()
  {
    if (m_dataTracker.hasChanged(d_R00)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R01)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R02)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R10)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R11)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R12)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R20)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R21)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_R22)) RotationMatrixDataChanged();
    if (m_dataTracker.hasChanged(d_tx)) TranslationVectorDataChanged();
    if (m_dataTracker.hasChanged(d_ty)) TranslationVectorDataChanged();
    if (m_dataTracker.hasChanged(d_tz)) TranslationVectorDataChanged();

    Matrix3 R;
    R[0][0] = d_R00.getValue();
    R[0][1] = d_R01.getValue();
    R[0][2] = d_R02.getValue();
    R[1][0] = d_R10.getValue();
    R[1][1] = d_R11.getValue();
    R[1][2] = d_R12.getValue();
    R[2][0] = d_R20.getValue();
    R[2][1] = d_R21.getValue();
    R[2][2] = d_R22.getValue();

    Vector3 t;
    t[0] = d_tx.getValue();
    t[1] = d_ty.getValue();
    t[2] = d_tz.getValue();

    l_cam->setPosition(t, false);
    l_cam->setRotationMatrix(R, false);
    l_cam->buildFromKRT();
  }

  virtual void handleEvent(sofa::core::objectmodel::Event* e)
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
      update();
    }
  }

  CamSettings l_cam;

  Vector3 m_pole;
  sofa::Data<double> d_thetaInit;
  sofa::Data<double> d_phiInit;
  sofa::Data<double> d_rhoInit;

  double m_theta;
  double m_phi;
  double m_rho;

  sofa::Data<double> d_tx;  ///< Position along x in world coordinates of the
                            /// camera's optical center
  sofa::Data<double> d_ty;  ///< Position along y in world coordinates of the
                            /// camera's optical center
  sofa::Data<double> d_tz;  ///< Position along z in world coordinates of the
                            /// camera's optical center

  sofa::Data<double> d_R00;  ///< 3x3 rotation matrix 00
  sofa::Data<double> d_R01;  ///< 3x3 rotation matrix 01
  sofa::Data<double> d_R02;  ///< 3x3 rotation matrix 02
  sofa::Data<double> d_R10;  ///< 3x3 rotation matrix 10
  sofa::Data<double> d_R11;  ///< 3x3 rotation matrix 10
  sofa::Data<double> d_R12;  ///< 3x3 rotation matrix 10
  sofa::Data<double> d_R20;  ///< 3x3 rotation matrix 20
  sofa::Data<double> d_R21;  ///< 3x3 rotation matrix 21
  sofa::Data<double> d_R22;  ///< 3x3 rotation matrix 22

  sofa::Data<std::string> d_imgS;  ///< serialized image

 private:
  void TranslationVectorDataChanged()
  {
    // setPositionData(d_tx.getValue(),d_ty.getValue(),d_tz.getValue());
  }
  void RotationMatrixDataChanged()
  {
    // setRotationMatrixData(d_R00.getValue(),d_R01.getValue(),d_R02.getValue(),d_R10.getValue(),d_R11.getValue(),d_R12.getValue(),d_R20.getValue(),d_R21.getValue(),d_R22.getValue());
  }
};

SOFA_DECL_CLASS(CameraController)

int CameraControllerClass =
    sofa::core::RegisterObject(
        "Component to rotate a camera following any trajectory on a sphere, "
        "around a point. Init sets the correct camera orientation if necessary")
        .add<CameraController>();

}  // namespace control
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
