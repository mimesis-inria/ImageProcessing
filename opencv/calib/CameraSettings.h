#ifndef SOFA_OR_PROCESSOR_CAMERASETTINGS_H
#define SOFA_OR_PROCESSOR_CAMERASETTINGS_H

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
class CameraSettings : public common::ImplicitDataEngine
{
	enum InputMode
	{
		P_AS_INPUT,            // pass the 3x4 proj mat
		CV_AS_INPUT,           // pass K, R, t
		CURRENT_GL_AS_INPUT,   // pass nothing, computes from OpenGL's context
		SPECIFIC_GL_AS_INPUT,  // pass glModelview, glProjection and glViewport
		SettingsMode_COUNT
	};

 public:
	SOFA_CLASS(CameraSettings, common::ImplicitDataEngine);

	CameraSettings()
			: d_mode(initData(
						&d_mode, "inputMode",
						"Choose between the following 3 options:\n"
						"  P_AS_INPUT: pass only the 3x4 projection matrix P as input\n"
						"  CV_AS_INPUT: pass OpenCV's K, R and t\n"
						"  CURRENT_GL_AS_INPUT: pass nothing, uses glGet() internally\n"
						"  SPECIFIC_GL_AS_INPUT: pass OpenGL stuff as input\n")),
				d_P(initData(&d_P, "P", "3x4 Projection matrix")),
				d_K(initData(&d_K, "K", "3x3 camera matrix from OpenCV")),
				d_R(initData(&d_R, "R", "3x3 rotation matrix")),
				d_t(initData(&d_t, "t", "transation vector")),
				d_glProjection(initData(&d_glProjection, "glProjection",
																"OpenGL's 4x4 Projection matrix")),
				d_glModelview(initData(&d_glModelview, "glModelview",
															 "OpenGL's 4x4 Modelview matrix")),
				d_glViewportWidth(initData(&d_glViewportWidth, -1, "viewportWidth",
																	 "opengl's viewport width")),
				d_glViewportHeight(initData(&d_glViewportHeight, -1, "viewportHeight",
																		"opengl's viewport height")),
				d_imageWidth(initData(&d_imageWidth, -1, "imageWidth", "image width")),
				d_imageHeight(
						initData(&d_imageHeight, -1, "imageHeight", "image height")),
				d_zNear(initData(&d_zNear, -1.0f, "zNear", "OpenGL's zNear")),
				d_zFar(initData(&d_zFar, -1.0f, "zFar", "OpenGL's zFar"))
	{
		sofa::helper::OptionsGroup* t = d_mode.beginEdit();
		t->setNames(SettingsMode_COUNT, "P_AS_INPUT", "CV_AS_INPUT",
								"CURRENT_GL_AS_INPUT", "SPECIFIC_GL_AS_INPUT");
		t->setSelectedItem("CV_AS_INPUT");
		d_mode.endEdit();
	}

	~CameraSettings() {}
	void init()
	{
		switch (d_mode.getValue().getSelectedId())
		{
			case P_AS_INPUT:
				addInput(&d_P);

				addOutput(&d_K);
				addOutput(&d_R);
				addOutput(&d_t);
				addOutput(&d_glProjection);
				addOutput(&d_glModelview);
				break;
			case CV_AS_INPUT:
				addInput(&d_K);
				addInput(&d_R);
				addInput(&d_t);

				addOutput(&d_P);
				addOutput(&d_glProjection);
				addOutput(&d_glModelview);
				break;
			case SPECIFIC_GL_AS_INPUT:
				addInput(&d_glProjection);
				addInput(&d_glModelview);

				addOutput(&d_P);
				addOutput(&d_K);
				addOutput(&d_R);
				addOutput(&d_t);
				break;
			case CURRENT_GL_AS_INPUT:
				addOutput(&d_glProjection);
				addOutput(&d_glModelview);
				addOutput(&d_P);
				addOutput(&d_K);
				addOutput(&d_R);
				addOutput(&d_t);
				break;
		}
		update();
	}

	void update()
	{
		switch (d_mode.getValue().getSelectedId())
		{
			case P_AS_INPUT:
			{
				decomposeProjection(d_P.getValue(), *d_K.beginEdit(), *d_R.beginEdit(),
														*d_t.beginEdit(), *d_glModelview.beginEdit(),
														*d_glProjection.beginEdit(),
														d_imageWidth.getValue(), d_imageHeight.getValue(),
														d_zNear.getValue(), d_zFar.getValue());
			}
			break;
			case CV_AS_INPUT:
			{
				assembleProjection(d_K.getValue(), d_R.getValue(), d_t.getValue(),
													 *d_P.beginEdit());
				decomposeProjection(d_P.getValue(), *d_K.beginEdit(), *d_R.beginEdit(),
														*d_t.beginEdit(), *d_glModelview.beginEdit(),
														*d_glProjection.beginEdit(),
														d_imageWidth.getValue(), d_imageHeight.getValue(),
														d_zNear.getValue(), d_zFar.getValue());
			}
			break;
			case CURRENT_GL_AS_INPUT:
			{
				// Constructs the camera matrices that allows to use
				// an image generated from the current OpenGL context for image
				// processing in OpenCV

				defaulttype::Matrix4 model;
				glGetDoublev(GL_MODELVIEW_MATRIX, &model[0][0]);

				defaulttype::Matrix4 proj;
				glGetDoublev(GL_PROJECTION_MATRIX, &proj[0][0]);

				d_glProjection.setValue(proj);
				d_glModelview.setValue(model);

				GLint viewport[4];
				glGetIntegerv(GL_VIEWPORT, viewport);

				assembleProjection(d_glProjection.getValue(), d_glModelview.getValue(),
													 *d_P.beginEdit(), viewport[2], viewport[3]);
				decomposeProjection(d_P.getValue(), *d_K.beginEdit(), *d_R.beginEdit(),
														*d_t.beginEdit(), *d_glModelview.beginEdit(),
														*d_glProjection.beginEdit(), viewport[2],
														viewport[3]);
			}
			break;
			case SPECIFIC_GL_AS_INPUT:
			{
				// Constructs the projection & calibration matrices that allows to use
				// an image generated from a specific OpenGL context for image
				// processing in OpenCV
				assembleProjection(d_glProjection.getValue(), d_glModelview.getValue(),
													 *d_P.beginEdit(), d_glViewportWidth.getValue(),
													 d_glViewportHeight.getValue());
				decomposeProjection(d_P.getValue(), *d_K.beginEdit(), *d_R.beginEdit(),
														*d_t.beginEdit(), *d_glModelview.beginEdit(),
														*d_glProjection.beginEdit(),
														d_glViewportWidth.getValue(),
														d_glViewportHeight.getValue());
			}
			break;
		}
	}

 private:
	// Build the 3x4 projection matrix from the 3x3 opencv camera matrix, R and t
	void assembleProjection(const defaulttype::Matrix3& cameraMatrix,
													const defaulttype::Matrix3& R,
													const defaulttype::Vec3d& t,
													defaulttype::Mat3x4d& projection);

	// Build the 3x4 projection matrix from the 3x3 opencv camera matrix, and 3x4
	// Rt matrix
	void assembleProjection(const defaulttype::Matrix3& cameraMatrix,
													const defaulttype::Mat3x4d& Rt,
													defaulttype::Mat3x4d& projection);

	// From a glProjection and a glModelview matrix, build the Projection matrix
	void assembleProjection(const defaulttype::Matrix4& glProjection,
													const defaulttype::Matrix4& glModelview,
													defaulttype::Mat3x4d& projection);

	// From a glProjection and a glModelview matrix, build the Projection matrix
	void assembleProjection(const defaulttype::Matrix4& glProjection,
													const defaulttype::Matrix4& glModelview,
													defaulttype::Mat3x4d& projection, int w, int h);

	// Decompose the Projection matrix into a calibration and a rotation matrix
	// and the position of a camera (t), and computes the OpenGL modelView and
	// Proejction matrices
	void decomposeProjection(const defaulttype::Mat3x4d& projection,
													 defaulttype::Matrix3& cameraMatrix,
													 defaulttype::Matrix3& rotation,
													 defaulttype::Vector3& translation,
													 defaulttype::Matrix4& glModelview,
													 defaulttype::Matrix4& glProjection, int w = -1,
													 int h = -1, float n = -1, float f = -1);

	Data<helper::OptionsGroup> d_mode;
	Data<defaulttype::Mat3x4d> d_P;
	Data<defaulttype::Matrix3> d_K;
	Data<defaulttype::Matrix3> d_R;
	Data<defaulttype::Vector3> d_t;
	Data<defaulttype::Matrix4> d_glProjection;
	Data<defaulttype::Matrix4> d_glModelview;
	Data<int> d_glViewportWidth;
	Data<int> d_glViewportHeight;
	Data<int> d_imageWidth;
	Data<int> d_imageHeight;
	Data<float> d_zNear;
	Data<float> d_zFar;
};

SOFA_DECL_CLASS(CameraSettings)

int CameraSettingsClass =
		core::RegisterObject(
				"Component computing opengl parameters from OpenCV calibration data & vice versa")
				.add<CameraSettings>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMERASETTINGS_H
