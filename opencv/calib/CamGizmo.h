#ifndef SOFA_OR_PROCESSOR_CAMGIZMO_H
#define SOFA_OR_PROCESSOR_CAMGIZMO_H

#include "initPlugin.h"

#include <SofaORCommon/CameraUtils.h>
#include <SofaORCommon/ImplicitDataEngine.h>

#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualParams.h>
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
class CamGizmo : public common::ImplicitDataEngine
{
	typedef typename defaulttype::Vector3 Vector3;

 public:
	SOFA_CLASS(CamGizmo, common::ImplicitDataEngine);

	CamGizmo()
			: d_camPos(initData(&d_camPos, "camPos",
													"Camera position & orientation (Rigid3D)")),
				d_P(initData(&d_P, "P", "Camera's Projection matrix"))
	{
	}

	~CamGizmo() {}
	void init()
	{
		addInput(&d_camPos);
		addInput(&d_P);
		update();
	}

	void draw(const core::visual::VisualParams* vparams)
	{
		Vector3 camera_X =
				d_camPos.getValue().getOrientation().rotate(Vector3(1, 0, 0));
		Vector3 camera_Y =
				d_camPos.getValue().getOrientation().rotate(Vector3(0, 1, 0));
		Vector3 camera_Z =
				d_camPos.getValue().getOrientation().rotate(Vector3(0, 0, -1));

		glColor4f(1, 0, 0, 1);
		glLineWidth(1);

		defaulttype::Vector3 p1, p2, p3, p4;
		common::camera::getCornersPosition(p1, p2, p3, p4, 1280, 720,
																			 d_P.getValue(), 1.0f);

		glBegin(GL_LINES);
		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		helper::gl::glVertexT(p1);
		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		helper::gl::glVertexT(p2);
		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		helper::gl::glVertexT(p3);
		helper::gl::glVertexT(d_camPos.getValue().getCenter());
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
				d_camPos.getValue().getCenter(),
				d_camPos.getValue().getCenter() + camera_X * 0.01, 0.001,
				defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
		vparams->drawTool()->drawArrow(
				d_camPos.getValue().getCenter(),
				d_camPos.getValue().getCenter() + camera_Y * 0.01, 0.001,
				defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
		vparams->drawTool()->drawArrow(
				d_camPos.getValue().getCenter(),
				d_camPos.getValue().getCenter() + camera_Z * 0.01, 0.001,
				defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
	}

	Data<defaulttype::RigidTypes::Coord> d_camPos;
	Data<defaulttype::Mat3x4d> d_P;
};

SOFA_DECL_CLASS(CamGizmo)

int CamGizmoClass =
		core::RegisterObject("Component drawing a camera gizmo for debugging")
				.add<CamGizmo>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMGIZMO_H
