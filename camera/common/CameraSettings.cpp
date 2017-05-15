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
	const Mat3x4d& P = d_P.getValue();
	double rx = P[0][0] * pt[0] + P[0][1] * pt[1] + P[0][2] * pt[2] + P[0][3];
	double ry = P[1][0] * pt[0] + P[1][1] * pt[1] + P[1][2] * pt[2] + P[1][3];
	double rz = P[2][0] * pt[0] + P[2][1] * pt[1] + P[2][2] * pt[2] + P[2][3];

	return Vector2(rx, ry) * 1.0 / rz;
}

defaulttype::Vector3 CameraSettings::get3DFrom2DPosition(double x, double y,
																												 float fz)
{
	Matrix3 K = d_K.getValue();
	Matrix3 R = d_R.getValue();
	Vector3 t = -R * d_t.getValue();

	Matrix3 iK;
	iK.invert(K);

	Vector3 point3D =
			iK * Vector3(x, y, 1) * ((fz == -1) ? (d_fz.getValue()) : (fz));

	return R.transposed() * (point3D - t);
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

const defaulttype::Mat3x4d& CameraSettings::getProjectionMatrix() const
{
	return d_P.getValue();
}
void CameraSettings::setProjectionMatrix(const Mat3x4d& P)
{
	d_P.setValue(P);
	decomposeP();
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
		composeP();
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
		composeP();
		composeGL();
	}
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vector3& CameraSettings::getTranslationVector() const
{
	return d_t.getValue();
}
void CameraSettings::setTranslationVector(const Vector3& t, bool update)
{
	d_t.setValue(t);
	if (update)
	{
		decomposeCV();
		composeP();
		composeGL();
	}
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vec2i& CameraSettings::getImageSize() const
{
	return d_imageSize.getValue();
}
void CameraSettings::setImageSize(const Vec2i& imgSize)
{
	d_imageSize.setValue(imgSize);
	composeP();
	composeGL();
	this->checkData(false);

	recalculate2DCorners();
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
	composeP();
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
	composeP();
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
const defaulttype::RigidTypes::Coord& CameraSettings::getCamPos() const
{
	return d_camPos.getValue();
}
void CameraSettings::setCamPos(const Rigid& camPos)
{
	d_camPos.setValue(camPos);
	composeP();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}

const defaulttype::Vector2& CameraSettings::getFocalLength() const
{
	return d_f.getValue();
}
void CameraSettings::setFocalLength(const Vector2& f)
{
	d_f.setValue(f);
	composeP();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}
float CameraSettings::getFz() const { return d_fz.getValue(); }
void CameraSettings::setFz(float fz)
{
	// No need to recompose, fz only used for projection operations
	d_fz.setValue(fz);
	this->checkData(false);

	recalculate3DCorners();
}
const defaulttype::Vector2& CameraSettings::getPrincipalPointPosition() const
{
	return d_c.getValue();
}
void CameraSettings::setPrincipalPointPosition(const Vector2& c)
{
	d_c.setValue(c);
	composeP();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
}
float CameraSettings::getAxisSkew() const { return d_s.getValue(); }
void CameraSettings::setAxisSkew(float s)
{
	d_s.setValue(s);
	composeP();
	composeGL();
	this->checkData(false);

	recalculate3DCorners();
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
	double fx = K[0][0];
	//	double s = K[0][1];
	double cx = K[0][2];

	double fy = K[1][1];
	double cy = K[1][2];

	d_f.setValue(defaulttype::Vector2(fx, fy));
	d_c.setValue(defaulttype::Vector2(cx, cy));
	//	d_s.setValue(s);
	d_s.setValue(0.0);

	Quat camera_ori;
	camera_ori.fromMatrix(R);

	Rigid cpos;
	cpos.getCenter() = t;
	cpos.getOrientation() = camera_ori;

	d_camPos.setValue(cpos);
}

// Decomposes P (sets f, c, s, camPos)
void CameraSettings::decomposeP()
{
	std::cout << "decomposeP:\nP = \n" << d_P.getValue();
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
	std::cout << "decomposeGL:\nModelview = \n"
						<< d_glModelview.getValue() << "\nProjection = \n"
						<< d_glProjection.getValue() << std::endl;
	Matrix4 glP = d_glProjection.getValue();
	Matrix4 glM = d_glModelview.getValue();

	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().y();

	double fx = 0.5 * w * glP[0][0];
	double fy = 0.5 * h * (1.0 + glP[1][1]);
	double cx = 0.5 * w * (1.0 - glP[0][3]);
	double cy = 0.5 * h * (1.0 + glP[1][3]);

	d_f.setValue(Vector2(fx, fy));
	d_s.setValue(0);
	d_c.setValue(Vector2(cx, cy));
	Matrix3 R;
	for (unsigned j = 0; j < 3; j++)
		for (unsigned i = 0; i < 3; i++) R[j][i] = glM[j][i];

	Quat camera_ori;
	camera_ori.fromMatrix(R);
	Vector3 camera_pos(glM[0][3], glM[1][3], glM[2][3]);

	Rigid cpos;
	cpos.getOrientation() = Quat(Vector3(1, 0, 0), M_PI) * camera_ori;
	cpos.getOrientation().toMatrix(R);
	cpos.getCenter() = -R * camera_pos;

	d_camPos.setValue(cpos);
}

// decomposes OpenCV's K, R and t
void CameraSettings::decomposeCV()
{
	std::cout << "decomposeCV:\nK = \n"
						<< d_K.getValue() << "\nR = \n"
						<< d_R.getValue() << "\nt = \n"
						<< d_t.getValue() << std::endl;

	decomposeKRt(d_K.getValue(), d_R.getValue(), d_t.getValue());
}

// Composes OpenGL's Modelview and Projection matrices
void CameraSettings::composeGL()
{
	std::cout << "composeGL" << std::endl;
	std::cout << d_f.getName() << d_f.getValue() << std::endl;
	std::cout << d_c.getName() << d_c.getValue() << std::endl;
	std::cout << d_camPos.getName()
						<< " center: " << d_camPos.getValue().getCenter() << std::endl;
	std::cout << d_camPos.getName()
						<< "orientation: " << d_camPos.getValue().getOrientation()
						<< std::endl;

	float n = d_zClip.getValue().x();
	float f = d_zClip.getValue().y();
	double w = d_imageSize.getValue().x();
	double h = d_imageSize.getValue().y();

	double fx = d_f.getValue().x();
	double fy = d_f.getValue().y();
	double cx = d_c.getValue().x();
	double cy = d_c.getValue().y();

	Matrix4 glP;

	glP[0][0] = 2.0 * fx / w;
	glP[0][1] = 0;
	glP[0][2] = 0;
	glP[0][3] = 1.0 - 2.0 * cx / w;

	//	glP[1][0] = -2.0 * s / w;
	glP[1][0] = 0;
	glP[1][1] = 2.0 * fy / h;
	glP[1][2] = 0;
	glP[1][3] = -1.0 + 2.0 * cy / h;

	glP[2][0] = 0;
	glP[2][1] = 0;
	glP[2][2] = (-(f + n)) / (f - n);
	glP[2][3] = (-2.0 * f * n) / (f - n);

	glP[3][0] = 0;
	glP[3][1] = 0;
	glP[3][2] = -1.0;
	glP[3][3] = 0;

	d_glProjection.setValue(glP);

	Matrix4 MM;

	Matrix3 R;
	Quat Orig = d_camPos.getValue().getOrientation();
	Orig.toMatrix(R);

	Vector3 p = R * d_camPos.getValue().getCenter();

	Orig = Quat(Vector3(1, 0, 0), M_PI) * d_camPos.getValue().getOrientation();
	Orig.toMatrix(R);

	for (unsigned int j = 0; j < 3; j++)
	{
		for (unsigned int i = 0; i < 3; i++) MM[j][i] = R[j][i];

		MM[j][3] = p[j];
		MM[3][j] = 0;
	}
	MM[3][3] = 1.0;

	d_glModelview.setValue(MM);
}

// Composes K, R, t
void CameraSettings::composeCV()
{
	std::cout << "composeCV" << std::endl;

	double fx = d_f.getValue().x();
	double fy = d_f.getValue().y();
	//	double s = d_s.getValue();
	double cx = d_c.getValue().x();
	double cy = d_c.getValue().y();

	Matrix3 K;
	// see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
	K[0][0] = fx;
	//	K[0][1] = s;
	K[0][1] = 0;
	K[0][2] = cx;

	K[1][0] = 0;
	K[1][1] = fy;
	K[1][2] = cy;

	K[2][0] = 0;
	K[2][1] = 0;
	K[2][2] = 1.0;
	d_K.setValue(K);

	Matrix3 R;
	Quat q = d_camPos.getValue().getOrientation();
	q.toMatrix(R);
	d_R.setValue(R);
	d_t.setValue(d_camPos.getValue().getCenter());
}

// Composes the Projection matrix P
void CameraSettings::composeP()
{
	std::cout << "composeP" << std::endl;
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

	// http://perception.inrialpes.fr/~Horaud/livre-fichiersPS/VO-HoraudMonga.pdf
	// p.146
	Mat3x4d M = K * Rt;

	d_P.setValue(M);
}

void CameraSettings::recalculate2DCorners()
{
	int w = d_imageSize.getValue().x();
	int h = d_imageSize.getValue().y();
	helper::vector<Vector2>& corners2D = *d_2DCorners.beginEdit();
	corners2D.push_back(Vector2(0, 0));
	corners2D.push_back(Vector2(0, h));
	corners2D.push_back(Vector2(w, h));
	corners2D.push_back(Vector2(w, 0));
	d_2DCorners.endEdit();
}

void CameraSettings::recalculate3DCorners()
{
	Vector3 p1, p2, p3, p4;
	this->getCornersPosition(p1, p2, p3, p4);
	helper::vector<Vector3>& corners3D = *d_3DCorners.beginEdit();
	corners3D.push_back(p4);
	corners3D.push_back(p1);
	corners3D.push_back(p2);
	corners3D.push_back(p3);
	d_3DCorners.endEdit();
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
	addDataCallback(&d_glViewport, (callback)&CameraSettings::GLViewportChanged);
	addDataCallback(&d_zClip, (callback)&CameraSettings::GLZClipChanged);
	addDataCallback(&d_camPos, (callback)&CameraSettings::CamPosChanged);
	addDataCallback(&d_f, (callback)&CameraSettings::FocalLengthChanged);
	addDataCallback(&d_fz, (callback)&CameraSettings::FzChanged);
	addDataCallback(&d_c,
									(callback)&CameraSettings::PrincipalPointPositionChanged);
	addDataCallback(&d_s, (callback)&CameraSettings::AxisSkewChanged);

	addOutput(&d_2DCorners);
	addOutput(&d_3DCorners);

	Matrix4 mv;

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
			std::cout << "CameraSettings: from GL" << std::endl;
			decomposeGL();
			composeP();
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
			composeP();
		}
	}
	recalculate2DCorners();

	recalculate3DCorners();

	checkData(false);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
