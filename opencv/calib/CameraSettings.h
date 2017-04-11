#ifndef SOFA_OR_PROCESSOR_CAMERASETTINGS_H
#define SOFA_OR_PROCESSOR_CAMERASETTINGS_H

#include "initPlugin.h"

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
	typedef defaulttype::Vec<4, double> Vec4d;
	typedef defaulttype::Vec<2, int> Vec2i;
	typedef defaulttype::Vec<5, float> Vector5;
	typedef defaulttype::Mat3x4d Mat3x4d;
	typedef defaulttype::Matrix4 Matrix4;
	typedef defaulttype::Matrix3 Matrix3;
	typedef defaulttype::Quat Quat;

 public:
	SOFA_CLASS(CameraSettings, common::ImplicitDataEngine);

	// CameraSettings ctor. All parameters are optional and can be set using
	// calibration components, Opengl components, or file loaders
	CameraSettings()
			: d_P(initData(&d_P, "P", "3x4 Projection matrix")),
				d_K(initData(&d_K, "K", "3x3 camera matrix from OpenCV")),
				d_distCoefs(initData(&d_distCoefs, "distCoefs",
														 "The camera's distortion coefficients")),
				d_R(initData(&d_R, "R", "3x3 rotation matrix")),
				d_t(initData(&d_t, "t", "translation vector")),
				d_imageSize(
						initData(&d_imageSize, "imageSize", "Image resolution in pixels")),
				d_glProjection(initData(&d_glProjection, "glProjection",
																"OpenGL's 4x4 Projection matrix")),
				d_glModelview(initData(&d_glModelview, "glModelview",
															 "OpenGL's 4x4 Modelview matrix")),
				d_zClip(initData(&d_zClip, Vector2(0.01f, 1000.0f), "zClip",
												 "OpenGL's z clipping values in scene unit")),
				d_camPos(initData(&d_camPos, "camPos", "3D position of the camera")),
				d_f(initData(&d_f, "f", "focal length")),
				d_fz(initData(&d_fz, "fz", "distance camera -> plane")),
				d_c(initData(&d_c, "c",
										 "principal point position in the image (in pixel units)")),
				d_s(initData(&d_s, "s",
										 "Axis skew (usually set to 0. Used in some very specific "
										 "digitalization processes)"))

	{
	}

	~CameraSettings() {}
	void init();
	void update() {}
	// returns the 2D pixel position of a given 3D point
	Vector2 get2DFrom3DPosition(const Vector3& p);

	// returns the 3D position of a 2D point 'x, y'
	Vector3 get3DFrom2DPosition(double x, double y, float fz = -1);
	Vector3 get3DFrom2DPosition(const Vector2& p, float fz = -1);

	// Returns the corners of the image plane
	void getCornersPosition(Vector3& p1, Vector3& p2, Vector3& p3, Vector3& p4,
													float fz = -1);

	// Getters & setters for the private Data (also used as callbacks when
	// modifying values in the GUI
	const Mat3x4d& getProjectionMatrix();
	void setProjectionMatrix(const Mat3x4d& P);

	const Matrix3& getIntrinsicCameraMatrix();
	void setIntrinsicCameraMatrix(const Matrix3& K);

	const helper::vector<double>& getDistortionCoefficients();
	void setDistortionCoefficients(const helper::vector<double>& distCoefs);

	const Matrix3& getRotationMatrix();
	void setRotationMatrix(const Matrix3& R);

	const Vector3& getTranslationVector();
	void setTranslationVector(const Vector3& t);

	const Vec2i& getImageSize();
	void setImageSize(const Vec2i& imgSize);
	const Matrix4& getGLProjection();
	void setGLProjection(const Matrix4& glProjection);

	const Matrix4& getGLModelview();
	void setGLModelview(const Matrix4& glModelview);

	const Vector2& getGLZClip();
	void setGLZClip(const Vector2& zClip);

	const Rigid& getCamPos();
	void setCamPos(const Rigid& camPos);

	const Vector2& getFocalLength();
	void setFocalLength(const Vector2& f);

	float getFz();
	void setFz(float fz);

	const Vec2i& getPrincipalPointPosition();
	void setPrincipalPointPosition(const Vector2& c);

	float getAxisSkew();
	void setAxisSkew(float s);

 private:
	void dumpValues()
	{
//		std::cout << "f: " << d_f.getValue() << " c: " << d_c.getValue()
//							<< " s: " << d_s.getValue() << std::endl;
	}

	Data<Mat3x4d> d_P;
	Data<Matrix3> d_K;
	Data<helper::vector<double> > d_distCoefs;
	Data<Matrix3> d_R;
	Data<Vector3> d_t;
	Data<Vec2i> d_imageSize;
	Data<Matrix4> d_glProjection;
	Data<Matrix4> d_glModelview;
	Data<Vector2> d_zClip;
	Data<Rigid> d_camPos;
	Data<Vector2> d_f;
	Data<float> d_fz;
	Data<Vector2> d_c;
	Data<float> d_s;

	// Decomposes P
	void decomposeP();

	// Decomposes OpenGL modelview and projection matrix
	void decomposeGL();

	// decomposes OpenCV's K, R and t
	void decomposeCV();

	// Composes OpenGL's Modelview and Projection matrices
	void composeGL();

	// Composes K, R, t and camPos
	void composeCV();

	// Composes the Projection matrix P
	void composeP();
	void decomposeKRt(const Matrix3& K, const Matrix3& R, const Vector3& t);

 public:
	// Data callbacks for GUI
	void ProjectionMatrixChanged(core::objectmodel::BaseObject*)
	{
		setProjectionMatrix(d_P.getValue());
		this->checkData(false);
	}
	void IntrinsicCameraMatrixChanged(core::objectmodel::BaseObject*)
	{
		setIntrinsicCameraMatrix(d_K.getValue());
		this->checkData(false);
	}
	void DistortionCoefficientsChanged(core::objectmodel::BaseObject*)
	{
		setDistortionCoefficients(d_distCoefs.getValue());
		this->checkData(false);
	}
	void RotationMatrixChanged(core::objectmodel::BaseObject*)
	{
		setRotationMatrix(d_R.getValue());
		this->checkData(false);
	}
	void TranslationVectorChanged(core::objectmodel::BaseObject*)
	{
		setTranslationVector(d_t.getValue());
		this->checkData(false);
	}
	void ImageSizeChanged(core::objectmodel::BaseObject*)
	{
		setImageSize(d_imageSize.getValue());
		this->checkData(false);
	}
	void GLProjectionChanged(core::objectmodel::BaseObject*)
	{
		setGLProjection(d_glProjection.getValue());
		this->checkData(false);
	}
	void GLModelviewChanged(core::objectmodel::BaseObject*)
	{
		setGLModelview(d_glModelview.getValue());
		this->checkData(false);
	}
	void GLZClipChanged(core::objectmodel::BaseObject*)
	{
		setGLZClip(d_zClip.getValue());
		this->checkData(false);
	}
	void CamPosChanged(core::objectmodel::BaseObject*)
	{
		setCamPos(d_camPos.getValue());
		this->checkData(false);
	}
	void FocalLengthChanged(core::objectmodel::BaseObject*)
	{
		setFocalLength(d_f.getValue());
		this->checkData(false);
	}
	void FzChanged(core::objectmodel::BaseObject*)
	{
		setFz(d_fz.getValue());
		this->checkData(false);
	}
	void PrincipalPointPositionChanged(core::objectmodel::BaseObject*)
	{
		setPrincipalPointPosition(d_c.getValue());
		this->checkData(false);
	}
	void AxisSkewChanged(core::objectmodel::BaseObject*)
	{
		setAxisSkew(d_s.getValue());
		this->checkData(false);
	}
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMERASETTINGS_H
