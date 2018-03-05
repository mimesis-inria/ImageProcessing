#include "CameraTrajectory.h"

sofaor::processor::CameraTrajectory::CameraTrajectory()
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

void sofaor::processor::CameraTrajectory::init()
{
    addInput(&d_center);
    addInput(&d_angle);
    addInput(&d_plane);

    if (!l_cam.get())
        msg_error(getName() + "::init()") << "Error: No camera link set. ";
}

void sofaor::processor::CameraTrajectory::Update()
{
  if (m_dataTracker.isDirty(d_center))
    centerChanged();
  if (m_dataTracker.isDirty(d_angle))
    angleChanged();
  if (m_dataTracker.isDirty(d_plane))
    planeChanged();

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

void sofaor::processor::CameraTrajectory::handleEvent(sofa::core::objectmodel::Event *e)
{
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
        update();
    }
}
