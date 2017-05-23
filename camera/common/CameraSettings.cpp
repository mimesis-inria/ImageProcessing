#include "CameraSettings.h"
#include <SofaORCommon/cvMatUtils.h>

#include <SofaBaseVisual/BaseCamera.h>

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
	const Mat3x4d& M = d_M.getValue();
	double rx = M[0][0] * pt[0] + M[0][1] * pt[1] + M[0][2] * pt[2] + M[0][3];
	double ry = M[1][0] * pt[0] + M[1][1] * pt[1] + M[1][2] * pt[2] + M[1][3];
	double rz = M[2][0] * pt[0] + M[2][1] * pt[1] + M[2][2] * pt[2] + M[2][3];

	return Vector2(rx, ry) * 1.0 / rz;
}

defaulttype::Vector3 CameraSettings::get3DFrom2DPosition(double x, double y,
																												 double f)
{
	Matrix3 K = d_K.getValue();
	Matrix3 R = d_R.getValue();
	Vector3 t = -R * d_t.getValue();

	Matrix3 iK;
	iK.invert(K);

	Vector3 point3D =
			iK * Vector3(x, y, 1) * ((f == -1) ? (d_f.getValue()) : (f));

	return R.transposed() * (point3D - t);
}

defaulttype::Vector3 CameraSettings::get3DFrom2DPosition(const Vector2& p,
																												 double f)
{
	return get3DFrom2DPosition(p[0], p[1], f);
}

void CameraSettings::getCornersPosition(Vector3& p1, Vector3& p2, Vector3& p3,
																				Vector3& p4, double f)
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

	p1 = get3DFrom2DPosition(0, 0, f);
	p2 = get3DFrom2DPosition(0, h, f);
	p3 = get3DFrom2DPosition(w, h, f);
	p4 = get3DFrom2DPosition(w, 0, f);
}

const defaulttype::Mat3x4d& CameraSettings::getProjectionMatrix() const
{
	return d_M.getValue();
}
void CameraSettings::setProjectionMatrix(const Mat3x4d& M)
{
	d_M.setValue(M);
	decomposeM();
	composeCV();
	composeGL();

	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Matrix3& CameraSettings::getIntrinsicCameraMatrix() const
{
	return d_K.getValue();
}
void CameraSettings::setIntrinsicCameraMatrix(const Matrix3& K, bool update)
{
	d_K.setValue(K);
	if (update)
	{
		decomposeCV();
		composeM();
		composeGL();
	}
	this->checkData(false);

	recalculate3DCorners();
}

const helper::vector<double>& CameraSettings::getDistortionCoefficients() const
{
	return d_distCoefs.getValue();
}
void CameraSettings::setDistortionCoefficients(
		const helper::vector<double>& distCoefs)
{
	// Nothing to do, distortion coefficients are not (yet?) taken into account in
	// OpenGL
	d_distCoefs.setValue(distCoefs);
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Matrix3& CameraSettings::getRotationMatrix() const
{
	return d_R.getValue();
}
void CameraSettings::setRotationMatrix(const Matrix3& R, bool update)
{
	d_R.setValue(R);
	if (update)
	{
		decomposeCV();
		composeM();
		composeGL();
	}
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vector3& CameraSettings::getPosition() const
{
	return d_t.getValue();
}
void CameraSettings::setPosition(const Vector3& t, bool update)
{
	d_t.setValue(t);
	if (update)
	{
		decomposeCV();
		composeM();
		composeGL();
	}
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vec2i& CameraSettings::getImageSize() const
{
	return d_imageSize.getValue();
}
void CameraSettings::setImageSize(const Vec2i& imgSize, bool update)
{
	d_imageSize.setValue(imgSize);
	if (update)
	{
		composeM();
		composeGL();
	}
	this->checkData(false);

	recalculate3DCorners();
}
const defaulttype::Matrix4& CameraSettings::getGLProjection() const
{
	return d_glProjection.getValue();
}
void CameraSettings::setGLProjection(const Matrix4& glProjection)
{
	d_glProjection.setValue(glProjection);
	decomposeGL();
	composeM();
	this->checkData(false);

	recalculate3DCorners();
}
const defaulttype::Matrix4& CameraSettings::getGLModelview() const
{
	return d_glModelview.getValue();
}
void CameraSettings::setGLModelview(const Matrix4& glModelview)
{
	d_glModelview.setValue(glModelview);
	decomposeGL();
	composeM();
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vec<4, int>& CameraSettings::getGLViewport() const
{
	return d_glViewport.getValue();
}
void CameraSettings::setGLViewport(const Vector4& glViewport)
{
	d_glViewport.setValue(glViewport);
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vector2& CameraSettings::getGLZClip() const
{
	return d_zClip.getValue();
}
void CameraSettings::setGLZClip(const Vector2& zClip)
{
	d_zClip.setValue(zClip);
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}
const defaulttype::Quat& CameraSettings::getOrientation() const
{
	return d_orientation.getValue();
}
void CameraSettings::setOrientation(const Quat& orientation)
{
	d_orientation.setValue(orientation);
	Matrix3 R;
	orientation.toMatrix(R);
	d_R.setValue(R);
	composeM();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Matrix3& CameraSettings::get2DScaleMatrix() const
{
	return d_scale2D.getValue();
}
void CameraSettings::set2DScaleMatrix(const Matrix3& scale2D)
{
	d_scale2D.setValue(scale2D);
	composeM();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}
double CameraSettings::getFocalDistance() const { return d_f.getValue(); }
void CameraSettings::setFocalDistance(double f)
{
	// No need to recompose, fz only used for projection operations
	d_f.setValue(f);
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Matrix3& CameraSettings::get2DTranslationMatrix() const
{
	return d_translate2D.getValue();
}
void CameraSettings::set2DTranslationMatrix(const Matrix3& translation2D)
{
	d_translate2D.setValue(translation2D);
	composeM();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}

void CameraSettings::decomposeK(const Matrix3& K)
{
	if (d_imageSize.getValue().x() == 0 || d_imageSize.getValue().y() == 0)
	{
		msg_error(getName() + "::decomposeP()")
				<< "Error: trying to decompose P while imageSize has not been set";
		return;
	}

	Matrix3 translate2D(Vector3(1, 0, K[0][2]), Vector3(0, 1, K[1][2]),
											Vector3(0, 0, 1));
	Matrix3 scale2D(Vector3(K[0][0], 0, 0), Vector3(0, K[1][1], 0),
									Vector3(0, 0, 1));

	d_scale2D.setValue(scale2D);
	d_translate2D.setValue(translate2D);
}

// Decomposes P (sets f, c, s, camPos)
void CameraSettings::decomposeM()
{
	cv::Mat_<double> M, K, R, t;
	common::matrix::sofaMat2cvMat(d_M.getValue(), M);

	cv::decomposeProjectionMatrix(M, K, R, t);

	Vector3 _t =
			Vector3(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)) *
			1.0 / t.at<double>(3, 0);

	Matrix3 _K, _R;
	common::matrix::cvMat2sofaMat(K, _K);
	common::matrix::cvMat2sofaMat(R, _R);
	decomposeK(_K);
	d_R.setValue(_R);
	d_t.setValue(_t);
}

// Decomposes OpenGL modelview and projection matrix
void CameraSettings::decomposeGL()
{
	Matrix4 glP = d_glProjection.getValue();
	Matrix4 glM = d_glModelview.getValue();

	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().y();

	double fx = 0.5 * w * glP[0][0];
	double fy = 0.5 * h * (1.0 + glP[1][1]);
	double cx = 0.5 * w * (1.0 - glP[0][3]);
	double cy = 0.5 * h * (1.0 + glP[1][3]);

	Matrix3 translate2D(Vector3(1, 0, cx), Vector3(0, 1, cy), Vector3(0, 0, 1));
	Matrix3 scale2D(Vector3(fx, 0, 0), Vector3(0, fy, 0), Vector3(0, 0, 1));

	d_translate2D.setValue(translate2D);
	d_translate2D.setValue(scale2D);

	Matrix3 R;
	for (unsigned j = 0; j < 3; j++)
		for (unsigned i = 0; i < 3; i++) R[j][i] = glM[j][i];

	Quat camera_ori;
	camera_ori.fromMatrix(R);
	Vector3 camera_pos(glM[0][3], glM[1][3], glM[2][3]);

	camera_ori = Quat(Vector3(1.0, 0.0, 0.0), M_PI) * camera_ori;
	camera_pos = -R * camera_pos;

	d_orientation.setValue(camera_ori);
	d_R.setValue(R);
	d_t.setValue(camera_pos);
}

// decomposes OpenCV's K, R and t
void CameraSettings::decomposeCV() { decomposeK(d_K.getValue()); }

// Composes OpenGL's Modelview and Projection matrices
void CameraSettings::composeGL()
{
	double n = d_zClip.getValue().x();
	double f = d_zClip.getValue().y();
	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().y();

	Matrix3 T = d_translate2D.getValue();
	Matrix3 S = d_scale2D.getValue();

	Matrix4 glP;

	glP[0][0] = 2.0 * S[0][0] / w;
	glP[0][1] = 0.0;
	glP[0][2] = 0.0;
	glP[0][3] = 1.0 - 2.0 * T[0][2] / w;

	//	glP[1][0] = -2.0 * s / w;
	glP[1][0] = 0.0;
	glP[1][1] = 2.0 * S[1][1] / h;
	glP[1][2] = 0.0;
	glP[1][3] = -1.0 + 2.0 * T[1][2] / h;

	glP[2][0] = 0.0;
	glP[2][1] = 0.0;
	glP[2][2] = (-(f + n)) / (f - n);
	glP[2][3] = (-2.0 * f * n) / (f - n);

	glP[3][0] = 0.0;
	glP[3][1] = 0.0;
	glP[3][2] = -1.0;
	glP[3][3] = 0.0;

	d_glProjection.setValue(glP);

	Matrix4 MM;

	Quat q;
	q.fromMatrix(d_R.getValue());
	q = Quat(Vector3(1.0, 0.0, 0.0), M_PI) * q;
	d_orientation.setValue(q);

	Matrix3 R;
	d_orientation.getValue().toMatrix(R);

	Vector3 p = -R * d_t.getValue();

	for (unsigned int j = 0; j < 3; j++)
	{
		for (unsigned int i = 0; i < 3; i++) MM[j][i] = R[j][i];

		MM[j][3] = p[j];
		MM[3][j] = 0.0;
	}
	MM[3][3] = 1.0;

	d_glModelview.setValue(MM);
}

// Composes K, R, t
void CameraSettings::composeCV()
{
	Matrix3 T = d_translate2D.getValue();
	Matrix3 S = d_scale2D.getValue();

	Matrix3 K = T * S;
	d_K.setValue(K);
}

// Composes the Projection matrix P
void CameraSettings::composeM()
{
	composeCV();
	Vector3 t = d_t.getValue();
	const Matrix3& R = d_R.getValue();
	const Matrix3& K = d_K.getValue();

	// gets the camera position from the position of the world's reference frame
	// in camera coordinates
	t = -R * t;

	double ptr2[12] = {R[0][0], R[0][1], R[0][2], t[0],    R[1][0], R[1][1],
										 R[1][2], t[1],    R[2][0], R[2][1], R[2][2], t[2]};
	Mat3x4d Rt(ptr2);

	Mat3x4d M = K * Rt;

	d_M.setValue(M);
}

void CameraSettings::recalculate3DCorners()
{
	if (d_K.getValue() == Matrix3())
		return;
	Vector3 p1, p2, p3, p4;
	this->getCornersPosition(p1, p2, p3, p4,
													 (d_f.getValue() != -1) ? (d_f.getValue()) : (1.0f));
	helper::vector<Vector3>& corners3D = *d_3DCorners.beginEdit();
	corners3D.clear();
	corners3D.push_back(p4);
	corners3D.push_back(p1);
	corners3D.push_back(p2);
	corners3D.push_back(p3);
	d_3DCorners.endEdit();
}

/// Projects a point p on a plane defined by a Point A and a normal n
defaulttype::Vector3 orthoProj(const defaulttype::Vector3& p,
															 const defaulttype::Vector3& A,
															 const defaulttype::Vector3& n)
{
	double lambda = ((A * n) - (p * n)) / (n * n);
	return p + lambda * n;
}

double length(const defaulttype::Vector3& a, const defaulttype::Vector3& b)
{
	return sqrt((a.x() - b.x()) * (a.x() - b.x()) +
							(a.y() - b.y()) * (a.y() - b.y()) +
							(a.z() - b.z()) * (a.z() - b.z()));
}

void CameraSettings::buildFromCamPosAndImageCorners()
{
	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().x();

	Vector3 A = d_3DCorners.getValue()[0];
	Vector3 B = d_3DCorners.getValue()[1];
	Vector3 D = d_3DCorners.getValue()[3];

	Vector3 c = d_t.getValue();

	double u0, v0, fu, fv, f, Xmm, Ymm;
	Vector3 O, n, AB, AD, x, y;

	AB = B - A;
	AD = D - A;
	n = AB.normalized().cross(AD.normalized());

	// Optical center projection
	O = orthoProj(c, A, n);

	// focal distance
	f = length(c, O);

	x = orthoProj(O, A, AD.normalized());
	y = orthoProj(O, A, AB.normalized());

	double xdist = length(x, A);
	double ydist = length(y, A);

	// image dimensions in world units
	Xmm = length(B, A);
	Ymm = length(D, A);

	// pixel size
	double Su = Xmm / w;
	double Sv = Ymm / h;

	// 2D scaling
	fu = f / Su;
	fv = f / Sv;

	u0 = xdist / Su;
	v0 = ydist / Sv;

	Matrix3 R(Vector3(AB.normalized()), Vector3(AD.normalized()), Vector3(n));

	Rigid cam;
	cam.getOrientation().fromMatrix(R);
	cam.getCenter() = c;

	std::cout << "camPos:" << c << std::endl;
	std::cout << "camOri:" << R << std::endl;
	std::cout << "u0:" << u0 << std::endl;
	std::cout << "v0:" << v0 << std::endl;
	std::cout << "fu:" << fu << std::endl;
	std::cout << "fv:" << fv << std::endl;
	std::cout << "f:" << f << std::endl;
}

void CameraSettings::init()
{
	typedef ImplicitDataEngine::DataCallback callback;
	addDataCallback(&d_M, (callback)&CameraSettings::ProjectionMatrixChanged);
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
	addDataCallback(&d_glViewport, (callback)&CameraSettings::GLViewportChanged);
	addDataCallback(&d_zClip, (callback)&CameraSettings::GLZClipChanged);
	addDataCallback(&d_orientation,
									(callback)&CameraSettings::OrientationChanged);
	addDataCallback(&d_scale2D, (callback)&CameraSettings::Scale2DChanged);
	addDataCallback(&d_f, (callback)&CameraSettings::FocalDistanceChanged);
	addDataCallback(&d_translate2D,
									(callback)&CameraSettings::Translation2DChanged);

	addOutput(&d_3DCorners);

	if (d_imageSize.getValue().x() && d_imageSize.getValue().y())
	{
		if (d_3DCorners.isSet() && d_t.isSet())
		{
			this->buildFromCamPosAndImageCorners();
			composeCV();
			composeM();
			composeGL();
		}
		if (d_M.isSet())
		{
			decomposeM();
			composeCV();
			composeGL();
		}
		else if (d_K.isSet() || d_R.isSet() || d_t.isSet())
		{
			decomposeCV();
			composeM();
			composeGL();
		}
		else if (d_glProjection.isSet() || d_glModelview.isSet())
		{
			decomposeGL();
			composeM();
		}
		else
		{
			Matrix4 p, m;
			std::cout << "initializing from Opengl's default settings" << std::endl;
			glGetDoublev(GL_PROJECTION_MATRIX, p.ptr());
			glGetDoublev(GL_MODELVIEW_MATRIX, m.ptr());
			d_glProjection.setValue(p);
			d_glModelview.setValue(m);
			decomposeGL();
			composeM();
		}
	}
	recalculate3DCorners();

	checkData(false);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
