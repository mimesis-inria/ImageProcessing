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

#include "CalibratedCamera.h"

namespace sofaor
{
namespace processor
{
namespace cam
{
SOFA_DECL_CLASS(CalibratedCamera)

int CalibratedCameraClass =
    sofa::core::RegisterObject(
        "Component setting the CameraSettings to the camera")
        .add<CalibratedCamera>();

CalibratedCamera::CalibratedCamera()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters")),
      d_freeCam(initData(
          &d_freeCam, true, "freeCam",
          "when true, camera's modelview is not set. when false, OpenGL's "
          "camera is overriden by the new params")),
      d_freeProj(initData(&d_freeProj, true, "freeProj",
                          "when true, camera's projection matrix is not set. "
                          "when false, OpenGL's "
                          "camera is overriden by the new params")),
      d_drawGizmo(
          initData(&d_drawGizmo, false, "drawGizmo",
                   "displays the camera's reference frame and projection cone"))
{
  m_storeMatrices = false;
}

void CalibratedCamera::init()
{
  if (!l_cam.get())
    msg_error(getName() + "::init()") << "Error: No camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";
}

void CalibratedCamera::update()
{

}

void CalibratedCamera::preDrawScene(sofa::core::visual::VisualParams *vparams)
{
  if (!d_freeProj.getValue())
  {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMultMatrixd(l_cam->getGLProjection().transposed().ptr());
  }
  if (!d_freeCam.getValue())
  {
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMultMatrixd(l_cam->getGLModelview().transposed().ptr());
  }

  sofa::defaulttype::Vec<4, int> v;
  glGetIntegerv(GL_VIEWPORT, v.ptr());
  l_cam->setGLViewport(v);

  if (m_storeMatrices)
  {
    sofa::defaulttype::Matrix4 p, m;
    glGetDoublev(GL_PROJECTION_MATRIX, p.ptr());
    glGetDoublev(GL_MODELVIEW_MATRIX, m.ptr());
    l_cam->setGLProjection(p.transposed());
    l_cam->setGLModelview(m.transposed());

    std::cout << "Displaying current Projection and Modelview "
                 "Matrices:\nProjection:\n"
              << p.transposed() << "\nModelview:\n"
              << m.transposed() << std::endl;
    m_storeMatrices = false;
  }
  if (d_drawGizmo.getValue())
  {
    sofa::defaulttype::Matrix3 R;
    sofa::defaulttype::Quat q = l_cam->getOrientation();
    q.toMatrix(R);

    Vector3 camPos = l_cam->getPosition();
    Vector3 camera_X = R.line(0).normalized();
    Vector3 camera_Y = R.line(1).normalized();
    Vector3 camera_Z = R.line(2).normalized();

    glColor4f(1, 0, 0, 1);
    glLineWidth(1);

    sofa::defaulttype::Vector3 p1, p2, p3, p4;
    l_cam->getCornersPosition(p1, p2, p3, p4);

    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor4f(0, 0, 1, 1);
    sofa::helper::gl::glVertexT(camPos);
    sofa::helper::gl::glVertexT(p1);
    sofa::helper::gl::glVertexT(camPos);
    sofa::helper::gl::glVertexT(p2);
    sofa::helper::gl::glVertexT(camPos);
    sofa::helper::gl::glVertexT(p3);
    sofa::helper::gl::glVertexT(camPos);
    sofa::helper::gl::glVertexT(p4);
    glEnd();

    glLineWidth(3);

    glBegin(GL_LINES);
    glColor4f(0, 0, 1, 1);
    sofa::helper::gl::glVertexT(p1);
    sofa::helper::gl::glVertexT(p2);
    sofa::helper::gl::glVertexT(p2);
    sofa::helper::gl::glVertexT(p3);
    sofa::helper::gl::glVertexT(p3);
    sofa::helper::gl::glVertexT(p4);
    sofa::helper::gl::glVertexT(p4);
    sofa::helper::gl::glVertexT(p1);
    glEnd();
    glEnable(GL_LIGHTING);

    vparams->drawTool()->drawArrow(
        camPos, camPos + camera_X * 0.01, 0.001,
        sofa::defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
    vparams->drawTool()->drawArrow(
        camPos, camPos + camera_Y * 0.01, 0.001,
        sofa::defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
    vparams->drawTool()->drawArrow(
        camPos, camPos + camera_Z * 0.01, 0.001,
        sofa::defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
  }
  if (!d_freeCam.getValue() && !d_freeProj.getValue())
  {
    if (l_cam->isXRay())
      glDepthRange(1, 0);
    else
      glDepthRange(0, 1);
  }
}

void CalibratedCamera::postDrawScene(sofa::core::visual::VisualParams *)
{
  if (!d_freeProj.getValue())
  {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  }
  if (!d_freeCam.getValue())
  {
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }
}

void CalibratedCamera::handleEvent(sofa::core::objectmodel::Event *e)
{
  if (sofa::core::objectmodel::KeyreleasedEvent::checkEventType(e))
  {
    sofa::core::objectmodel::KeyreleasedEvent *kre =
        static_cast<sofa::core::objectmodel::KeyreleasedEvent *>(e);
    char keyPressed = kre->getKey();

    if (keyPressed == 'u' || keyPressed == 'U') m_storeMatrices = true;
  }
  ImplicitDataEngine::handleEvent(e);
}

void CalibratedCamera::computeBBox(const sofa::core::ExecParams *params, bool)
{
  sofa::helper::vector<sofa::defaulttype::Vector3> x = l_cam->getCorners();
  x.push_back(l_cam->getPosition());
  if (x.empty()) return;

  double minBBox[3] = {std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max()};
  double maxBBox[3] = {-std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max(),
                       -std::numeric_limits<double>::max()};

  for (unsigned int i = 0; i < x.size(); i++)
  {
    const sofa::defaulttype::Vector3 &p = x[i];
    for (int c = 0; c < 3; c++)
    {
      if (p[c] > maxBBox[c]) maxBBox[c] = p[c];
      if (p[c] < minBBox[c]) minBBox[c] = p[c];
    }
  }
  this->f_bbox.setValue(
      params, sofa::defaulttype::TBoundingBox<double>(minBBox, maxBBox));
}

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
