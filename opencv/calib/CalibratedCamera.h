#ifndef SOFA_OR_PROCESSOR_CALIBRATEDCAMERA_H
#define SOFA_OR_PROCESSOR_CALIBRATEDCAMERA_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "CameraSettings.h"

#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualManager.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/gl/Transformation.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glu.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class CalibratedCamera : public common::ImplicitDataEngine,
												 public core::visual::VisualManager
{
	typedef sofa::core::objectmodel::SingleLink<CalibratedCamera, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

	typedef typename defaulttype::Vector3 Vector3;

 public:
	SOFA_CLASS2(CalibratedCamera, common::ImplicitDataEngine,
							core::visual::VisualManager);

	CalibratedCamera()
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
				d_drawGizmo(initData(
						&d_drawGizmo, false, "drawGizmo",
						"displays the camera's reference frame and projection cone"))
	{
	}

	~CalibratedCamera() {}
	void init()
	{
		if (!l_cam.get())
			msg_error(getName() + "::init()") << "Error: No camera link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";
	}

	void preDrawScene(core::visual::VisualParams* vparams)
	{
		if (!d_freeProj.getValue())
		{
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			helper::gl::glMultMatrix(l_cam->getGLProjection().ptr());
		}
		if (!d_freeCam.getValue())
		{
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			helper::gl::glMultMatrix(l_cam->getGLModelview().ptr());
		}
		if (d_drawGizmo.getValue())
		{
			defaulttype::RigidTypes::Coord camPos = l_cam->getCamPos();
			Vector3 camera_X = camPos.getOrientation().rotate(Vector3(1, 0, 0));
			Vector3 camera_Y = camPos.getOrientation().rotate(Vector3(0, 1, 0));
			Vector3 camera_Z = camPos.getOrientation().rotate(Vector3(0, 0, -1));

			glColor4f(1, 0, 0, 1);
			glLineWidth(1);

			defaulttype::Vector3 p1, p2, p3, p4;
			l_cam->getCornersPosition(p1, p2, p3, p4);

			glBegin(GL_LINES);
			helper::gl::glVertexT(camPos.getCenter());
			helper::gl::glVertexT(p1);
			helper::gl::glVertexT(camPos.getCenter());
			helper::gl::glVertexT(p2);
			helper::gl::glVertexT(camPos.getCenter());
			helper::gl::glVertexT(p3);
			helper::gl::glVertexT(camPos.getCenter());
			helper::gl::glVertexT(p4);
			glEnd();

			glLineWidth(3);

			glBegin(GL_LINES);
			helper::gl::glVertexT(p1);
			helper::gl::glVertexT(p2);
			helper::gl::glVertexT(p2);
			helper::gl::glVertexT(p3);
			helper::gl::glVertexT(p3);
			helper::gl::glVertexT(p4);
			helper::gl::glVertexT(p4);
			helper::gl::glVertexT(p1);
			glEnd();

			vparams->drawTool()->drawArrow(
					camPos.getCenter(), camPos.getCenter() + camera_X * 0.01, 0.001,
					defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
			vparams->drawTool()->drawArrow(
					camPos.getCenter(), camPos.getCenter() + camera_Y * 0.01, 0.001,
					defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
			vparams->drawTool()->drawArrow(
					camPos.getCenter(), camPos.getCenter() + camera_Z * 0.01, 0.001,
					defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
		}
	}

	void postDrawScene(core::visual::VisualParams* /*vp*/)
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

	CamSettings l_cam;
	Data<bool> d_freeCam;
	Data<bool> d_freeProj;
	Data<bool> d_drawGizmo;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CALIBRATEDCAMERA_H
