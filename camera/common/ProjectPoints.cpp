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
