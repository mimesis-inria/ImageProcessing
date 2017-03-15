#ifndef SOFA_OR_PROCESSOR_CAMERAPROJECTION_H
#define SOFA_OR_PROCESSOR_CAMERAPROJECTION_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

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
class CameraProjection : public common::ImplicitDataEngine,
											 public core::visual::VisualManager
{
 public:
	SOFA_CLASS2(CameraProjection, common::ImplicitDataEngine, core::visual::VisualManager);

	CameraProjection()
			: d_freeCam(initData(&d_freeCam, true, "freeCam",
													 "set to false if you DON'T want the modelview matrix "
													 "to be overriden by this component")),
				d_glProjection(initData(&d_glProjection, "glProjection",
																"OpenGL's 4x4 Projection matrix")),
				d_glModelview(initData(&d_glModelview, "glModelview",
															 "OpenGL's 4x4 Modelview matrix"))
	{
	}

	~CameraProjection() {}
	void init()
	{
		addInput(&d_glProjection);
		addInput(&d_glModelview);
		update();
	}

	// HACK: ONCE SOFA WILL ACTUALLY USE THE DRAWTOOLS TO RENDER STUFF, THIS WILL
	// BE COMPLETELY OBSOLETE
	void preDrawScene(core::visual::VisualParams* /*vp*/)
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		helper::gl::glMultMatrix(d_glProjection.getValue().ptr());

		if (!d_freeCam.getValue())
		{
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			helper::gl::glMultMatrix(d_glModelview.getValue().ptr());
		}
	}

	// HACK: ONCE SOFA WILL ACTUALLY USE THE DRAWTOOLS TO RENDER STUFF, THIS WILL
	// BE COMPLETELY OBSOLETE
	void postDrawScene(core::visual::VisualParams* /*vp*/)
	{
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		if (!d_freeCam.getValue())
		{
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();
		}
	}

	Data<bool> d_freeCam;
	Data<defaulttype::Matrix4> d_glProjection;
	Data<defaulttype::Matrix4> d_glModelview;
};

SOFA_DECL_CLASS(CameraProjection)

int CameraProjectionClass =
		core::RegisterObject(
				"Component setting opengl's projection / modelview matrices")
				.add<CameraProjection>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMERAPROJECTION_H
