#ifndef SOFA_OR_PROCESSOR_CAMERASETTINGS_H
#define SOFA_OR_PROCESSOR_CAMERASETTINGS_H

#include "initPlugin.h"

#include <SofaORCommon/CameraUtils.h>
#include <SofaORCommon/ImplicitDataEngine.h>

#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualManager.h>
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
class CameraSettings : public common::ImplicitDataEngine
{
 public:
	typedef defaulttype::RigidTypes::Coord Rigid;
	typedef defaulttype::Vector2 Vector2;
	typedef defaulttype::Vector3 Vector3;
	typedef defaulttype::Vec<5, float> Vector5;
	typedef defaulttype::Mat3x4d Mat3x4d;
	typedef defaulttype::Matrix4 Matrix4;
	typedef defaulttype::Matrix3 Matrix3;
	typedef defaulttype::Quat Quat;

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
				d_P_out(initData(&d_P_out, "P_out", "out 3x4 Projection matrix")),
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
				d_zFar(initData(&d_zFar, -1.0f, "zFar", "OpenGL's zFar")),
				d_camPos(initData(&d_camPos, "camPos", "3D position of the camera"))
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

				addOutput(&d_P_out);
				addOutput(&d_K);
				addOutput(&d_R);
				addOutput(&d_t);
				addOutput(&d_glProjection);
				addOutput(&d_glModelview);
				addOutput(&d_camPos);
				break;
			case CV_AS_INPUT:
				addInput(&d_K);
				addInput(&d_R);
				addInput(&d_t);

				addOutput(&d_P);
				addOutput(&d_glProjection);
				addOutput(&d_glModelview);
				addOutput(&d_camPos);
				break;
			case SPECIFIC_GL_AS_INPUT:
				addInput(&d_glProjection);
				addInput(&d_glModelview);

				addOutput(&d_P);
				addOutput(&d_K);
				addOutput(&d_R);
				addOutput(&d_t);
				addOutput(&d_camPos);
				break;
			case CURRENT_GL_AS_INPUT:
				addOutput(&d_glProjection);
				addOutput(&d_glModelview);
				addOutput(&d_P);
				addOutput(&d_K);
				addOutput(&d_R);
				addOutput(&d_t);
				addOutput(&d_camPos);
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
				setCamera(d_P.getValue(), d_imageWidth.getValue(),
									d_imageHeight.getValue(), d_zNear.getValue(),
									d_zFar.getValue());
				//				decomposeProjection(d_P.getValue(),
				//*d_K.beginEdit(), *d_R.beginEdit(),
				//														*d_t.beginEdit(),
				//*d_glModelview.beginEdit(),
				//														*d_glProjection.beginEdit(),
				//														d_imageWidth.getValue(),
				// d_imageHeight.getValue(),
				//														d_zNear.getValue(),
				// d_zFar.getValue());
				//				d_P_out.setValue(d_P.getValue());
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
				d_P_out.setValue(d_P.getValue());
			}
			break;
			case CURRENT_GL_AS_INPUT:
			{
				// Constructs the camera matrices that allows to use
				// an image generated from the current OpenGL context for image
				// processing in OpenCV

				Matrix4 model;
				glGetDoublev(GL_MODELVIEW_MATRIX, &model[0][0]);

				Matrix4 proj;
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
				d_P_out.setValue(d_P.getValue());
			}
			break;
			case SPECIFIC_GL_AS_INPUT:
			{
				// Constructs the projection & calibration matrices that allows to use
				// an image generated from a specific OpenGL context for image
				// processing in OpenCV

				setCamera(d_glProjection.getValue(), d_glModelview.getValue(),
									d_imageWidth.getValue(), d_imageHeight.getValue());

				//				assembleProjection(d_glProjection.getValue(),
				//d_glModelview.getValue(),
				//													 *d_P.beginEdit(),
				//d_glViewportWidth.getValue(),
				//													 d_glViewportHeight.getValue());
				//				decomposeProjection(d_P.getValue(),
				//*d_K.beginEdit(), *d_R.beginEdit(),
				//														*d_t.beginEdit(),
				//*d_glModelview.beginEdit(),
				//														*d_glProjection.beginEdit(),
				//														d_glViewportWidth.getValue(),
				//														d_glViewportHeight.getValue());
				//				d_P_out.setValue(d_P.getValue());
			}
			break;
		}
	}

	void draw(const core::visual::VisualParams* vparams)
	{
		//		Vector3 camera_X =
		//				d_camPos.getValue().getOrientation().rotate(Vector3(1,
		//0,
		// 0));
		//		Vector3 camera_Y =
		//				d_camPos.getValue().getOrientation().rotate(Vector3(0,
		//1,
		// 0));
		//		Vector3 camera_Z =
		//				d_camPos.getValue().getOrientation().rotate(Vector3(0,
		//0,
		// 1));

		//		glColor4f(1, 0, 0, 1);
		//		glLineWidth(1);

		//		defaulttype::Vector3 p1, p2, p3, p4;
		//		common::camera::getCornersPosition(p1, p2, p3, p4,
		// d_imageWidth.getValue(),
		//																			 d_imageHeight.getValue(),
		//																			 d_P_out.getValue(),
		// 1.0f);

		//		glBegin(GL_LINES);
		//		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		//		helper::gl::glVertexT(p1);
		//		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		//		helper::gl::glVertexT(p2);
		//		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		//		helper::gl::glVertexT(p3);
		//		helper::gl::glVertexT(d_camPos.getValue().getCenter());
		//		helper::gl::glVertexT(p4);
		//		glEnd();

		//		glLineWidth(3);

		//		glBegin(GL_LINES);
		//		helper::gl::glVertexT(p1);
		//		helper::gl::glVertexT(p2);
		//		helper::gl::glVertexT(p2);
		//		helper::gl::glVertexT(p3);
		//		helper::gl::glVertexT(p3);
		//		helper::gl::glVertexT(p4);
		//		helper::gl::glVertexT(p4);
		//		helper::gl::glVertexT(p1);
		//		glEnd();

		//		vparams->drawTool()->drawArrow(
		//				d_camPos.getValue().getCenter(),
		//				d_camPos.getValue().getCenter() + camera_X *
		//0.01,
		// 0.001,
		//				defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
		//		vparams->drawTool()->drawArrow(
		//				d_camPos.getValue().getCenter(),
		//				d_camPos.getValue().getCenter() + camera_Y *
		//0.01,
		// 0.001,
		//				defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
		//		vparams->drawTool()->drawArrow(
		//				d_camPos.getValue().getCenter(),
		//				d_camPos.getValue().getCenter() + camera_Z *
		//0.01,
		// 0.001,
		//				defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0f));
	}

 private:
	// Computes the modelview and projection matrix from the intrinsic parameters
	// (focal, aperture, znear zfar, and image size) and the camera position /
	// orientation
	void assembleOpenGL(Matrix4& glProjection, Matrix4& glModelview,
											const Vector3& camera_pos, const Quat& camera_ori,
											double fx, double fy, double s, double x0, double y0,
											double w, double h, double n, double f);

	// Build the 3x4 projection matrix from the 3x3 opencv camera matrix, R and t
	void assembleProjection(const Matrix3& cameraMatrix, const Matrix3& R,
													const Vector3& t, Mat3x4d& projection);

	// Build the 3x4 projection matrix from the 3x3 opencv camera matrix, and 3x4
	// Rt matrix
	void assembleProjection(const Matrix3& cameraMatrix, const Mat3x4d& Rt,
													Mat3x4d& projection);

	// From a glProjection and a glModelview matrix, build the Projection matrix
	void assembleProjection(const Matrix4& glProjection,
													const Matrix4& glModelview, Mat3x4d& projection);

	// From a glProjection and a glModelview matrix, build the Projection matrix
	void assembleProjection(const Matrix4& glProjection,
													const Matrix4& glModelview, Mat3x4d& projection,
													unsigned w, unsigned h);

	// Decompose the Projection matrix into a calibration and a rotation matrix
	// and the position of a camera (t), and computes the OpenGL modelView and
	// Proejction matrices
	void decomposeProjection(const Mat3x4d& projection, Matrix3& cameraMatrix,
													 Matrix3& rotation, Vector3& translation,
													 Matrix4& glModelview, Matrix4& glProjection,
													 unsigned w = -1, unsigned h = -1, float n = -1,
													 float f = -1);

	// Decomposes M to get pinhole camera model params
	void setCamera(const Mat3x4d& M, unsigned w, unsigned h, float zNear,
								 float zFar)
	{
		CvMat* cvM = cvCreateMat(3, 4, CV_32F);
		CvMat* cvK = cvCreateMat(3, 3, CV_32F);
		CvMat* cvR = cvCreateMat(3, 3, CV_32F);
		CvMat* cvT = cvCreateMat(4, 1, CV_32F);

		for (unsigned j = 0; j < 3; j++)
		{
			for (unsigned i = 0; i < 4; i++)
			{
				cvmSet(cvM, j, i, M[j][i]);
			}
		}

		cvDecomposeProjectionMatrix(cvM, cvK, cvR, cvT);

		// see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
		double fx = 2.0 * cvmGet(cvK, 0, 0) / w;
		double s = -2.0 * cvmGet(cvK, 0, 1) / w;
		double x0 =
				(w - 2.0 * cvmGet(cvK, 0, 2) /*+ 2.0*d_oglCenter.getValue()[0]*/) / w;

		double fy = 2.0 * cvmGet(cvK, 1, 1) / h;
		double y0 = (-1.0 * h +
								 2.0 * cvmGet(cvK, 1, 2) /*+ 2.0*d_oglCenter.getValue()[1]*/) /
								h;

		Vector3 camera_pos =
				Vector3(cvmGet(cvT, 0, 0), cvmGet(cvT, 1, 0), cvmGet(cvT, 2, 0)) * 1.0 /
				cvmGet(cvT, 3, 0);

		Matrix3 R;
		for (unsigned j = 0; j < 3; j++)
		{
			for (unsigned i = 0; i < 3; i++)
			{
				R[j][i] = cvmGet(cvR, j, i);
			}
		}
		Quat camera_ori;
		camera_ori.fromMatrix(R.transposed());
		camera_ori *= Quat(Vector3(0, 1, 0), -M_PI) * Quat(Vector3(0, 0, 1), -M_PI);

		cvReleaseMat(&cvM);
		cvReleaseMat(&cvK);
		cvReleaseMat(&cvR);
		cvReleaseMat(&cvT);

		setCamera(camera_pos, camera_ori, fx, fy, s, x0, y0, w, h, zNear, zFar);
	}

	// get glModelview and glProjection from the decomposed M matrix
	void setCamera(Vector3 camera_pos, Quat camera_ori, double fx, double fy,
								 double s, double x0, double y0, unsigned w, unsigned h,
								 float n, float f)
	{
		Matrix4 MP;

		MP[0][0] = fx;
		MP[1][0] = s;
		MP[2][0] = x0;
		MP[3][0] = 0;

		MP[0][1] = 0;
		MP[1][1] = fy;
		MP[2][1] = y0;
		MP[3][1] = 0;

		MP[0][2] = 0;
		MP[1][2] = 0;
		MP[2][2] = -(f + n) / (f - n);
		MP[3][2] = (-2.0 * n * f) / (f - n);

		MP[0][3] = 0;
		MP[1][3] = 0;
		MP[2][3] = -1;
		MP[3][3] = 0;

		d_glProjection.setValue(MP);

		Matrix4 MM;

		Matrix3 R;
		camera_ori.toMatrix(R);
		Matrix3 iR = R.transposed();

		Vector3 p = -iR * camera_pos;

		for (unsigned int j = 0; j < 3; j++)
		{
			for (unsigned int i = 0; i < 3; i++)
			{
				MM[j][i] = R[j][i];
			}

			MM[3][j] = p[j];
			MM[j][3] = 0;
		}
		MM[3][3] = 1.0;

		d_glModelview.setValue(MM);

		setCamera(d_glProjection.getValue(), d_glModelview.getValue(), w, h);
	}

	// get M, cameraPos & intrinsicParamsVec from glValues
	void setCamera(const defaulttype::Matrix4& mp, const defaulttype::Matrix4& mm,
								 unsigned w, unsigned h)
	{
		double fx = mp[0][0];
		double s = mp[1][0];
		double x0 = mp[2][0];
		double fy = mp[1][1];
		double y0 = mp[2][1];

		Matrix3 R;
		for (unsigned j = 0; j < 3; j++)
		{
			for (unsigned i = 0; i < 3; i++)
			{
				R[j][i] = mm[j][i];
			}
		}

		Vector2 oglCenter(0.0, 0.0);
		Matrix3 K;
		// see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
		K[0][0] = 0.5 * w * fx;
		K[0][1] = -0.5 * w * s;
		K[0][2] = -0.5 * (w * x0 - 2.0 * oglCenter[0] - w);

		K[1][1] = 0.5 * h * fy;
		K[1][2] = 0.5 * (h * y0 - 2.0 * oglCenter[1] + h);

		K[2][2] = 1.0;

		Quat camera_ori;
		camera_ori.fromMatrix(R);
		Vector3 p(mm[3][0], mm[3][1], mm[3][2]);
		Vector3 camera_pos = -R * p;

		Quat Orig = camera_ori * Quat(Vector3(0, 0, 1), M_PI) *
								Quat(Vector3(0, 1, 0), M_PI);
		Matrix3 Rq;
		Orig.toMatrix(Rq);

		Matrix3 C = K * Rq.transposed();
		Vector3 T = -C * camera_pos;

		Mat3x4d M;

		for (unsigned j = 0; j < 3; j++)
		{
			for (unsigned i = 0; i < 3; i++)
			{
				M[j][i] = C[j][i];
			}
			M[j][3] = T[j];
		}

		d_P_out.setValue(M);

		Rigid cpos;

		cpos.getCenter() = camera_pos;
		cpos.getOrientation() = camera_ori;

		d_camPos.setValue(cpos);

		//    std::cout << "pos " << d_cameraPos.getValue() << std::endl;

		Vector5 intr;
		intr[0] = fx;
		intr[1] = fy;
		intr[2] = s;
		intr[3] = x0;
		intr[4] = y0;

		d_intrinsicParameters.setValue(intr);

		sout << "projectionMatrixOpenGL=\"" << d_glProjection.getValue() << "\""
				 << sendl;
		sout << "modelViewMatrixOpenGL=\"" << d_glModelview.getValue() << "\""
				 << sendl;
		sout << "projectionMatrix=\"" << d_P_out.getValue() << "\"" << sendl;
	}

	Data<helper::OptionsGroup> d_mode;
	Data<defaulttype::Mat3x4d> d_P;
	Data<defaulttype::Mat3x4d> d_P_out;
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
	Data<Rigid> d_camPos;
	Data<Vector5> d_intrinsicParameters;
};

SOFA_DECL_CLASS(CameraSettings)

int CameraSettingsClass = core::RegisterObject(
															"Component computing opengl parameters from "
															"OpenCV calibration data & vice versa")
															.add<CameraSettings>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMERASETTINGS_H
