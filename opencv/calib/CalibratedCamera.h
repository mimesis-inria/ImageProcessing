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
				d_freeCam(initData(&d_freeCam, true, "freeCam",
													 "when true, camera is not set. when false, OpenGL's "
													 "camera is overriden by the new params"))
	{
	}

	~CalibratedCamera() {}
	void init() { update(); }

	void preDrawScene(core::visual::VisualParams* /*vp*/)
	{
		if (!d_freeCam.getValue())
		{
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			helper::gl::glMultMatrix(l_cam->getGLProjection().ptr());

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			helper::gl::glMultMatrix(l_cam->getGLModelview().ptr());
		}
	}

	void postDrawScene(core::visual::VisualParams* /*vp*/)
	{
		if (!d_freeCam.getValue())
		{
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();

			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}
	}

	Data<bool> d_freeCam;
	CamSettings l_cam;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CALIBRATEDCAMERA_H
