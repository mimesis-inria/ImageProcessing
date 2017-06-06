#include "StereoSettings.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(StereoSettings)

int StereoSettingsClass =
		core::RegisterObject(
				"Stereo camera settings component whose task is to store and maintain "
				"stereoscopic camera parameters up to date")
				.add<StereoSettings>();

cv::Mat_<double> StereoSettings::iterativeLinearLSTriangulation(cv::Point3d u,
																																cv::Point3d u1)
{
	double wi = 1, wi1 = 1;
	cv::Mat_<double> X(4, 1);

	cv::Mat_<double> X_ = linearLSTriangulation(u, u1);
	X(0) = X_(0);
	X(1) = X_(1);
	X(2) = X_(2);
	X(3) = 1.0;

	for (int i = 0; i < 10; i++)
	{  // Hartley suggests 10 iterations at most
		// recalculate weights
		double p2x = cv::Mat_<double>(cv::Mat_<double>(P1).row(2) * X)(0);
		double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P2).row(2) * X)(0);

		// breaking point
		// if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

		wi = p2x;
		wi1 = p2x1;

		// reweight equations and solve
		cv::Matx43d A(
				(u.x * P1(2, 0) - P1(0, 0)) / wi, (u.x * P1(2, 1) - P1(0, 1)) / wi,
				(u.x * P1(2, 2) - P1(0, 2)) / wi, (u.y * P1(2, 0) - P1(1, 0)) / wi,
				(u.y * P1(2, 1) - P1(1, 1)) / wi, (u.y * P1(2, 2) - P1(1, 2)) / wi,
				(u1.x * P2(2, 0) - P2(0, 0)) / wi1, (u1.x * P2(2, 1) - P2(0, 1)) / wi1,
				(u1.x * P2(2, 2) - P2(0, 2)) / wi1, (u1.y * P2(2, 0) - P2(1, 0)) / wi1,
				(u1.y * P2(2, 1) - P2(1, 1)) / wi1, (u1.y * P2(2, 2) - P2(1, 2)) / wi1);

		cv::Mat_<double> B =
				(cv::Mat_<double>(4, 1) << -(u.x * P1(2, 3) - P1(0, 3)) / wi,
				 -(u.y * P1(2, 3) - P1(1, 3)) / wi, -(u1.x * P2(2, 3) - P2(0, 3)) / wi1,
				 -(u1.y * P2(2, 3) - P2(1, 3)) / wi1);

		cv::solve(A, B, X_, cv::DECOMP_SVD);
		X(0) = X_(0);
		X(1) = X_(1);
		X(2) = X_(2);
		X(3) = 1.0;
	}

	return X;
}

cv::Mat_<double> StereoSettings::linearLSTriangulation(cv::Point3d u,
																											 cv::Point3d u1)
{
	/**
	From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image
	understanding, 1997
	*/
	cv::Matx43d A(u.x * P1(2, 0) - P1(0, 0), u.x * P1(2, 1) - P1(0, 1),
								u.x * P1(2, 2) - P1(0, 2), u.y * P1(2, 0) - P1(1, 0),
								u.y * P1(2, 1) - P1(1, 1), u.y * P1(2, 2) - P1(1, 2),
								u1.x * P2(2, 0) - P2(0, 0), u1.x * P2(2, 1) - P2(0, 1),
								u1.x * P2(2, 2) - P2(0, 2), u1.y * P2(2, 0) - P2(1, 0),
								u1.y * P2(2, 1) - P2(1, 1), u1.y * P2(2, 2) - P2(1, 2));

	cv::Matx41d B(-(u.x * P1(2, 3) - P1(0, 3)), -(u.y * P1(2, 3) - P1(1, 3)),
								-(u1.x * P2(2, 3) - P2(0, 3)), -(u1.y * P2(2, 3) - P2(1, 3)));

	cv::Mat_<double> X;
	cv::solve(A, B, X, cv::DECOMP_SVD);

	return X;
}

void StereoSettings::init()
{
	if (l_cam1.get() && l_cam2.get() && !d_F.isSet() && !d_E.isSet())
		recomputeFromCameras();
}

// returns the 3D position of a pair of 2D matches 'X, Y'
defaulttype::Vector3 StereoSettings::triangulate(const Vector2& x1,
																								 const Vector2& x2)
{
	common::matrix::sofaMat2cvMat(l_cam1->getProjectionMatrix(), P1);
	common::matrix::sofaMat2cvMat(l_cam2->getProjectionMatrix(), P2);

	cv::Point3d u(x1.x(), x2.y(), 1.0);
	cv::Point3d u1(x2.x(), x2.y(), 1.0);

//	// multiply the point by the inverse of the K matrix
//	cv::Mat_<double> um = cm1.inv() * cv::Mat_<double>(u);
//	cv::Mat_<double> um1 = cm2.inv() * cv::Mat_<double>(u1);

//	u.x = um(0);
//	u.y = um(1);
//	u1.x = um1(0);
//	u1.y = um1(1);
//	u.z = um(2);
//	u1.z = um1(2);

	cv::Mat_<double> X = iterativeLinearLSTriangulation(u, u1);

	return Vector3(X(0), X(1), X(2));
}

// returns the 3D position of a pair of 2D matches 'X, Y'
void StereoSettings::triangulate(const Vector2& x1, const Vector2& x2,
																 Vector3& w)
{
	w = triangulate(x1, x2);
}

// Getters & setters for the private Data (also used as callbacks when
// modifying values in the GUI
const defaulttype::Matrix3& StereoSettings::getFundamentalMatrix()
{
	return d_F.getValue();
}

void StereoSettings::setFundamentalMatrix(const Matrix3& F) { d_F.setValue(F); }
const defaulttype::Matrix3& StereoSettings::getEssentialMatrix()
{
	return d_E.getValue();
}
void StereoSettings::setEssentialMatrix(const Matrix3& E) { d_E.setValue(E); }
//void StereoSettings::updateRt()
//{
//	const Vector3& t = d_t.getValue();
//	const Matrix3& R = d_R.getValue();
//	P1 = cv::Matx34d::eye();
//	P2 = cv::Matx34d(R[0][0], R[0][1], R[0][2], t[0], R[1][0], R[1][1], R[1][2],
//									 t[1], R[2][0], R[2][1], R[2][2], t[2]);
//}

//const defaulttype::Matrix3& StereoSettings::getRotationMatrix()
//{
//	return d_R.getValue();
//}
//void StereoSettings::setRotationMatrix(const Matrix3& R)
//{
//	d_R.setValue(R);
//	updateRt();
//}
//const defaulttype::Vector3& StereoSettings::getTranslationVector()
//{
//	return d_t.getValue();
//}

//void StereoSettings::setTranslationVector(const Vector3& t)
//{
//	d_t.setValue(t);
//	updateRt();
//}

void StereoSettings::recomputeFromCameras()
{
	if (!l_cam1.get() || !l_cam2.get())
		return;

	Matrix3 K1 = l_cam1->getIntrinsicCameraMatrix();
	Matrix3 K2 = l_cam2->getIntrinsicCameraMatrix();
	Matrix3 R1 = l_cam1->getRotationMatrix();
	Matrix3 R2 = l_cam2->getRotationMatrix();
	Vector3 T1 = l_cam1->getPosition();
	Vector3 T2 = l_cam2->getPosition();

	if (K1 == Matrix3() || K2 == Matrix3())
		return;

	// Camera 2 in Cam1 coord system
//	Vector3 Ts = R1 * (T2 - T1);
	Vector3 Ts = T2 - T1;

	// Rotation matrix from R1 to R2
	Matrix3 K2R2 = K2 * R2;
	K2R2.invert(K2R2.transposed());

	// Skew symetric matrix of Ts
	Matrix3 T(
				Vector3(0, -Ts[2], Ts[1]),
			Vector3(Ts[2], 0, -Ts[0]),
			Vector3(-Ts[1], Ts[0], 0));

	/// E = R * [T]x
	Matrix3 E;

	/// F = inv(K2') * R * [T]x * inv(K1)
	Matrix3 KTmp = K1;
	K1.invert(KTmp);

	Matrix3 K1R1 = R1.transposed() * K1;

	Matrix3 F = K2R2 * T * K1R1;

	// "Normalize matrix
	for (int i = 0 ; i < 3 ; ++i)
	{
		for (int j = 0 ; j < 3 ; ++j)
		{
			F[i][j] /= F[2][2];
		}
	}

	this->setFundamentalMatrix(F);
	this->setEssentialMatrix(E);

}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
