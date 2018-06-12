#include "ProjectPoints.h"

namespace sofacv
{
namespace cam
{
SOFA_DECL_CLASS(ProjectPoints)

int ProjectPointsClass = sofa::core::RegisterObject(
                             "Component projecting points from 2D to 3D & vice "
                             "versa using a linked CameraSettings")
                             .add<ProjectPoints>();

ProjectPoints::ProjectPoints()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters")),
      d_2Dto3D(initData(&d_2Dto3D, true, "to3D", "if false, 3D to 2D")),
      d_depth(
          initData(&d_depth, -1.0, "depth",
                   "default is -1 (retrieves depth from fz in camSettings)")),
      d_Pts3D(initData(&d_Pts3D, "points3D", "3D points")),
      d_Pts2D(initData(&d_Pts2D, "points2D", "2D points"))
{
}

void ProjectPoints::init()
{
  if (!l_cam.get())
    msg_error(getName() + "::init()") << "Error: No camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";
  if (d_2Dto3D.getValue())
  {
    addInput(&d_Pts2D);
    addOutput(&d_Pts3D);
  }
  else
  {
    addInput(&d_Pts3D);
    addOutput(&d_Pts2D);
  }
  update();
}

void ProjectPoints::Update()
{
  if (d_2Dto3D.getValue())
  {
    sofa::helper::vector<Vector3>& pts3d = *d_Pts3D.beginEdit();
    pts3d.clear();
    for (auto pt : d_Pts2D.getValue())
      pts3d.push_back(l_cam->get3DFrom2DPosition(pt, d_depth.getValue()));
  }
  else
  {
    sofa::helper::vector<Vector2>& pts2d = *d_Pts2D.beginEdit();
    pts2d.clear();
    for (auto pt : d_Pts3D.getValue())
      pts2d.push_back(l_cam->get2DFrom3DPosition(pt));
  }
}

}  // namespace cam
}  // namespace sofacv
