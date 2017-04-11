#include "CameraSettings.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(CameraSettings)

int CameraSettingsClass =
		core::RegisterObject(
				"Camera settings component whose task is to store and maintain every "
				"camera parameters up to date, and perform the basic projective "
				"transformations")
				.add<CameraSettings>();

defaulttype::Vector2 CameraSettings::get2DFrom3DPosition(const Vector3& pt)
{
	const Mat3x4d& P = d_P.getValue();
	double rx = P[0][0] * pt[0] + P[0][1] * pt[1] + P[0][2] * pt[2] + P[0][3];
	double ry = P[1][0] * pt[0] + P[1][1] * pt[1] + P[1][2] * pt[2] + P[1][3];
	double rz = P[2][0] * pt[0] + P[2][1] * pt[1] + P[2][2] * pt[2] + P[2][3];

	return Vector2(rx, ry) * 1.0 / rz;
}

defaulttype::Vector3 CameraSettings::get3DFrom2DPosition(double x, double y,
																												 float fz)
{
	Matrix3 C;
	Vector3 T;

	const Mat3x4d& P = d_P.getValue();
	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 3; i++)
		{
			C[j][i] = P[j][i];
		}
		T[j] = P[j][3];
	}

	if (defaulttype::oneNorm(C) == 0) return Vector3();

	Matrix3 iC;
	iC.invert(C);

	Vector3 camera_pos = -iC * T;

	return iC * Vector3(x, y, 1) * ((fz == -1) ? (d_fz.getValue()) : (fz)) +
				 camera_pos;
}

defaulttype::Vector3 CameraSettings::get3DFrom2DPosition(const Vector2& p,
																												 float fz)
{
	return get3DFrom2DPosition(p[0], p[1], fz);
}

void CameraSettings::getCornersPosition(Vector3& p1, Vector3& p2, Vector3& p3,
																				Vector3& p4, float fz)
{
	if (d_imageSize.getValue().x() == 0 || d_imageSize.getValue().y() == 0)
	{
		msg_error(getName() + "::getCornersPosition()")
				<< "Error: trying to project the image's corners while imageSize has "
					 "not been set";
		return;
	}

	int w = d_imageSize.getValue().x();
	int h = d_imageSize.getValue().y();
	p1 = get3DFrom2DPosition(0, 0, fz);
	p2 = get3DFrom2DPosition(0, h, fz);
	p3 = get3DFrom2DPosition(w, h, fz);
	p4 = get3DFrom2DPosition(w, 0, fz);
}

const defaulttype::Mat3x4d& CameraSettings::getProjectionMatrix()
{
	return d_P.getValue();
}
void CameraSettings::setProjectionMatrix(const Mat3x4d& P)
{
	d_P.setValue(P);
	decomposeP();
	composeCV();
	composeGL();

	this->clean();
}

const defaulttype::Matrix3& CameraSettings::getIntrinsicCameraMatrix()
{
	return d_K.getValue();
}
void CameraSettings::setIntrinsicCameraMatrix(const Matrix3& K)
{
	d_K.setValue(K);
	decomposeCV();
	composeP();
	composeGL();
}

const helper::vector<double>& CameraSettings::getDistortionCoefficients()
{
	return d_distCoefs.getValue();
}
void CameraSettings::setDistortionCoefficients(
		const helper::vector<double>& distCoefs)
{
	// Nothing to do, distortion coefficients are not (yet?) taken into account in
	// OpenGL
	d_distCoefs.setValue(distCoefs);
}

const defaulttype::Matrix3& CameraSettings::getRotationMatrix()
{
	return d_R.getValue();
}
void CameraSettings::setRotationMatrix(const Matrix3& R)
{
	d_R.setValue(R);
	decomposeCV();
	composeP();
	composeGL();
}

const defaulttype::Vector3& CameraSettings::getTranslationVector()
{
	return d_t.getValue();
}
void CameraSettings::setTranslationVector(const Vector3& t)
{
	d_t.setValue(t);
	decomposeCV();
	composeP();
	composeGL();
}

const defaulttype::Vec2i& CameraSettings::getImageSize()
{
	return d_imageSize.getValue();
}
void CameraSettings::setImageSize(const Vec2i& imgSize)
{
	d_imageSize.setValue(imgSize);
	composeP();
	composeGL();
}
const defaulttype::Matrix4& CameraSettings::getGLProjection()
{
	return d_glProjection.getValue();
}
void CameraSettings::setGLProjection(const Matrix4& glProjection)
{
	d_glProjection.setValue(glProjection);
	decomposeGL();
	composeP();
}
const defaulttype::Matrix4& CameraSettings::getGLModelview()
{
	return d_glModelview.getValue();
}
void CameraSettings::setGLModelview(const Matrix4& glModelview)
{
	d_glModelview.setValue(glModelview);
	decomposeGL();
	composeP();
}
const defaulttype::Vector2& CameraSettings::getGLZClip()
{
	return d_zClip.getValue();
}
void CameraSettings::setGLZClip(const Vector2& zClip)
{
	d_zClip.setValue(zClip);
	composeGL();
}
const defaulttype::RigidTypes::Coord& CameraSettings::getCamPos()
{
	return d_camPos.getValue();
}
void CameraSettings::setCamPos(const Rigid& camPos)
{
	d_camPos.setValue(camPos);
	composeP();
	composeGL();
}

const defaulttype::Vector2& CameraSettings::getFocalLength()
{
	return d_f.getValue();
}
void CameraSettings::setFocalLength(const Vector2& f)
{
	d_f.setValue(f);
	composeP();
	composeGL();
}
float CameraSettings::getFz() { return d_fz.getValue(); }
void CameraSettings::setFz(float fz)
{
	// No need to recompose, fz only used for projection operations
	d_fz.setValue(fz);
}
const defaulttype::Vec2i& CameraSettings::getPrincipalPointPosition()
{
	return d_c.getValue();
}
void CameraSettings::setPrincipalPointPosition(const Vector2& c)
{
	d_c.setValue(c);
	composeP();
	composeGL();
}
float CameraSettings::getAxisSkew() { return d_s.getValue(); }
void CameraSettings::setAxisSkew(float s)
{
	d_s.setValue(s);
	composeP();
	composeGL();
}
void CameraSettings::decomposeKRt(const Matrix3& K, const Matrix3& R,
																	const Vector3& t)
{
	if (d_imageSize.getValue().x() == 0 || d_imageSize.getValue().y() == 0)
	{
		msg_error(getName() + "::decomposeP()")
				<< "Error: trying to decompose P while imageSize has not been set";
		return;
	}

	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().y();

	// see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
	double fx = 2.0 * K[0][0] / w;
	double s = -2.0 * K[0][1] / w;
	double cx = (w - 2.0 * K[0][2]) / w;

	double fy = 2.0 * K[1][1] / h;
	double cy = (-1.0 * h + 2.0 * K[1][2]) / h;

	d_f.setValue(defaulttype::Vector2(fx, fy));
	d_c.setValue(defaulttype::Vector2(cx, cy));
	d_s.setValue(s);
	d_s.setValue(0.0);

	Matrix3 rotation;
	for (unsigned j = 0; j < 3; j++)
		for (unsigned i = 0; i < 3; i++) rotation[j][i] = R[j][i];

	Quat camera_ori;
	camera_ori.fromMatrix(rotation.transposed());
	camera_ori *= Quat(Vector3(0, 1, 0), -M_PI) * Quat(Vector3(0, 0, 1), -M_PI);

	Rigid cpos;
	cpos.getCenter() = t;
	cpos.getOrientation() = camera_ori;

	d_camPos.setValue(cpos);
}

// Decomposes P (sets f, c, s, camPos)
void CameraSettings::decomposeP()
{
	dumpValues();
	cv::Mat_<double> P, K, R, t;
	common::matrix::sofaMat2cvMat(d_P.getValue(), P);

	cv::decomposeProjectionMatrix(P, K, R, t);

	Vector3 _t =
			Vector3(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)) *
			1.0 / t.at<double>(3, 0);

	Matrix3 _K, _R;
	common::matrix::cvMat2sofaMat(K, _K);
	common::matrix::cvMat2sofaMat(R, _R);
	decomposeKRt(_K, _R, _t);
}

// Decomposes OpenGL modelview and projection matrix
void CameraSettings::decomposeGL()
{
	Matrix4 glP = d_glProjection.getValue();
	Matrix4 glM = d_glModelview.getValue();

	d_f.setValue(Vector2(glP[0][0], glP[1][1]));
	d_s.setValue(glP[1][0]);
	d_s.setValue(0);
	d_c.setValue(Vector2(glP[2][0], glP[2][1]));
	Matrix3 R;
	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 3; i++)
		{
			R[j][i] = glM[j][i];
		}
	}

	d_R.setValue(R);

	Quat camera_ori;
	camera_ori.fromMatrix(R);
	Vector3 p(glM[3][0], glM[3][1], glM[3][2]);
	Vector3 camera_pos = -R * p;
	Quat Orig =
			camera_ori * Quat(Vector3(0, 0, 1), M_PI) * Quat(Vector3(0, 1, 0), M_PI);
	Rigid cpos;
	cpos.getCenter() = camera_pos;
	cpos.getOrientation() = camera_ori;

	d_camPos.setValue(cpos);
}

// decomposes OpenCV's K, R and t
void CameraSettings::decomposeCV()
{
	decomposeKRt(d_K.getValue(), d_R.getValue(), d_t.getValue());
}

// Composes OpenGL's Modelview and Projection matrices
void CameraSettings::composeGL()
{
	float n = d_zClip.getValue().x();
	float f = d_zClip.getValue().y();

	Matrix4 glP;

	glP[0][0] = d_f.getValue().x();
	glP[1][0] = d_s.getValue();
	glP[1][0] = 0;
	glP[2][0] = d_c.getValue().x();
	glP[3][0] = 0;

	glP[0][1] = 0;
	glP[1][1] = d_f.getValue().y();
	glP[2][1] = d_c.getValue().y();
	glP[3][1] = 0;

	glP[0][2] = 0;
	glP[1][2] = 0;
	glP[2][2] = -(f + n) / (f - n);
	glP[3][2] = (-2.0 * n * f) / (f - n);

	glP[0][3] = 0;
	glP[1][3] = 0;
	glP[2][3] = -1;
	glP[3][3] = 0;

	d_glProjection.setValue(glP);

	Matrix4 MM;

	Matrix3 R;
	d_camPos.getValue().getOrientation().toMatrix(R);
	Matrix3 iR = R.transposed();

	Vector3 p = -iR * d_camPos.getValue().getCenter();

	for (unsigned int j = 0; j < 3; j++)
	{
		for (unsigned int i = 0; i < 3; i++) MM[j][i] = R[j][i];

		MM[3][j] = p[j];
		MM[j][3] = 0;
	}
	// negate to follow OpenGL's right hand rule
	MM[3][3] = 1.0;

	d_glModelview.setValue(MM);
}

// Composes K, R, t and camPos
void CameraSettings::composeCV()
{
	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().y();
	double fx = d_f.getValue().x();
	double fy = d_f.getValue().y();
	double s = d_s.getValue();
	double cx = d_c.getValue().x();
	double cy = d_c.getValue().y();

	Vector2 oglCenter(0.0, 0.0);
	Matrix3 K;
	// see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
	K[0][0] = 0.5 * w * fx;
	K[0][1] = -0.5 * w * s;
	K[0][1] = 0;
	K[0][2] = -0.5 * (w * cx - 2.0 * oglCenter[0] - w);

	K[1][0] = 0;
	K[1][1] = 0.5 * h * fy;
	K[1][2] = 0.5 * (h * cy - 2.0 * oglCenter[1] + h);

	K[2][0] = 0;
	K[2][1] = 0;
	K[2][2] = 1.0;
	d_K.setValue(K);

	Matrix3 R;
	d_camPos.getValue().getOrientation().toMatrix(R);
	d_R.setValue(R);
	d_t.setValue(d_camPos.getValue().getCenter());
}

// Composes the Projection matrix P
void CameraSettings::composeP()
{
	composeCV();
	const Matrix3& R = d_R.getValue();
	const Vector3& t = d_t.getValue();
	Mat3x4d Rt = Mat3x4d(Vec4d(R[0][0], R[0][1], R[0][2], t[0]),
											 Vec4d(R[1][0], R[1][1], R[1][2], t[1]),
											 Vec4d(R[2][0], R[2][1], R[2][2], t[2]));

	d_P.setValue(d_K.getValue() * Rt);
}

void CameraSettings::init()
{
	typedef ImplicitDataEngine::DataCallback callback;
	addDataCallback(&d_P, (callback)&CameraSettings::ProjectionMatrixChanged);
	addDataCallback(&d_K,
									(callback)&CameraSettings::IntrinsicCameraMatrixChanged);
	addDataCallback(&d_distCoefs,
									(callback)&CameraSettings::DistortionCoefficientsChanged);
	addDataCallback(&d_R, (callback)&CameraSettings::RotationMatrixChanged);
	addDataCallback(&d_t, (callback)&CameraSettings::TranslationVectorChanged);
	addDataCallback(&d_imageSize, (callback)&CameraSettings::ImageSizeChanged);
	addDataCallback(&d_glProjection,
									(callback)&CameraSettings::GLProjectionChanged);
	addDataCallback(&d_glModelview,
									(callback)&CameraSettings::GLModelviewChanged);
	addDataCallback(&d_zClip, (callback)&CameraSettings::GLZClipChanged);
	addDataCallback(&d_camPos, (callback)&CameraSettings::CamPosChanged);
	addDataCallback(&d_f, (callback)&CameraSettings::FocalLengthChanged);
	addDataCallback(&d_fz, (callback)&CameraSettings::FzChanged);
	addDataCallback(&d_c,
									(callback)&CameraSettings::PrincipalPointPositionChanged);
	addDataCallback(&d_s, (callback)&CameraSettings::AxisSkewChanged);

	if (d_imageSize.getValue().x() && d_imageSize.getValue().y())
	{
		if (d_P.isSet())
		{
			decomposeP();
			composeCV();
			composeGL();
		}
		else if (d_K.isSet() || d_R.isSet() || d_t.isSet())
		{
			decomposeCV();
			composeP();
			composeGL();
		}
		else if (d_glProjection.isSet() || d_glModelview.isSet())
		{
			decomposeGL();
			composeP();
		}
	}
	checkData(false);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
