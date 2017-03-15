#include "CameraSettings.h"

#include <Eigen/Dense>
#include <Eigen/LU>

namespace sofa
{
namespace OR
{
namespace processor
{
using namespace defaulttype;

// Build the 3x4 projection matrix from the 3x3 opencv camera matrix, R and t
void CameraSettings::assembleProjection(const Matrix3& cameraMatrix,
																				const Matrix3& R, const Vector3& t,
																				Mat3x4d& projection)
{
	Mat3x4d Rt = Mat3x4d(Vec4d(R[0][0], R[0][1], R[0][2], t[0]),
											 Vec4d(R[1][0], R[1][1], R[1][2], t[1]),
											 Vec4d(R[2][0], R[2][1], R[2][2], t[2]));

	assembleProjection(cameraMatrix, Rt, projection);
}

// Build the 3x4 projection matrix from the 3x3 opencv camera matrix, and 3x4
// Rt matrix
void CameraSettings::assembleProjection(const Matrix3& cameraMatrix,
																				const Mat3x4d& Rt, Mat3x4d& projection)
{
	/////////////////////
	/// P = K * [R|t] ///
	/////////////////////

	projection = cameraMatrix * Rt;
}

// From a glProjection and a glModelview matrix, build the Projection matrix
void CameraSettings::assembleProjection(const Matrix4& glProjection, const Matrix4& glModelview,
												Mat3x4d& projection)
{
	// retrieve Opengl's viewport
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	assembleProjection(glProjection, glModelview, projection, viewport[2],
										 viewport[3]);
}

// From a glProjection, a glModelview matrix and the pixel dimensions of the
// viewport, build the Projection matrix
void CameraSettings::assembleProjection(const Matrix4& glProjection, const Matrix4& glModelview,
												Mat3x4d& projection, int w, int h)
{
	double fx = glProjection[0][0];
	double s = glProjection[1][0];
	double x0 = glProjection[2][0];
	double fy = glProjection[1][1];
	double y0 = glProjection[2][1];

	Matrix3 R;
	for (unsigned j = 0; j < 3; j++)
		for (unsigned i = 0; i < 3; i++) R[j][i] = glModelview[j][i];

	Vec2f oglCenter(0.0, 0.0);
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
	Vector3 p(glModelview[3][0], glModelview[3][1], glModelview[3][2]);
	Vector3 camera_pos = -R * p;

	Quat Orig =
			camera_ori * Quat(Vector3(0, 0, 1), M_PI) * Quat(Vector3(0, 1, 0), M_PI);
	Matrix3 Rq;
	Orig.toMatrix(Rq);

	Matrix3 C = K * Rq.transposed();
	Vector3 T = -C * camera_pos;

	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 3; i++) projection[j][i] = C[j][i];
		projection[j][3] = T[j];
	}
}

// Decompose the Projection matrix into a calibration and a rotation matrix
// and the position of a camera (t), and computes the OpenGL modelView and
// Proejction matrices. If w and h are not set, glModelView and glProjection
// won't be computed
void CameraSettings::decomposeProjection(
		const Mat3x4d& projection, Matrix3& cameraMatrix, Matrix3& rotation,
		Vector3& translation, Matrix4& glModelview, Matrix4& glProjection, int w,
		int h, float n, float f)
{
	cv::Mat_<double> K, R, t;
	cv::Matx34d M(projection.ptr());

	cv::decomposeProjectionMatrix(M, K, R, t);

	cameraMatrix = reinterpret_cast<double*>(K.data);
	rotation = reinterpret_cast<double*>(K.data);
	translation = reinterpret_cast<double*>(K.data);

	///////////////////////////////////////////
	/// Computing OpenGL's Modelview matrix ///
	///////////////////////////////////////////

	Vector3 camera_pos = Vector3(t[0][0], t[1][0], t[2][0]) * 1.0 / t[3][0];

	Matrix3 iR = rotation.transposed();

	Quat camera_ori;
	camera_ori.fromMatrix(iR);
	camera_ori *= Quat(Vector3(0, 1, 0), -M_PI) * Quat(Vector3(0, 0, 1), -M_PI);

	Matrix3 Ro;
	camera_ori.toMatrix(Ro);
	Matrix3 iRo = Ro.transposed();
	Vector3 p = -iRo * camera_pos;

	for (unsigned int j = 0; j < 3; j++)
	{
		for (unsigned int i = 0; i < 3; i++) glModelview[j][i] = Ro[j][i];

		glModelview[3][j] = p[j];
		glModelview[j][3] = 0;
	}
	glModelview[3][3] = 1.0;

	// we need the image's dimensions to compute the opengl projection matrix
	if (w == -1 || h == -1 || n == -1 || f == -1) return;

	////////////////////////////////////////////
	/// Computing OpenGL's Projection matrix ///
	////////////////////////////////////////////

	// see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
	double fx = 2.0 * K[0][0] / w;
	double s = -2.0 * K[0][1] / w;
	double x0 = (w - 2.0 * K[0][2]) / w;

	double fy = 2.0 * K[1][1] / h;
	double y0 = (-1.0 * h + 2.0 * K[1][2]) / h;

	glProjection[0][0] = fx;
	glProjection[1][0] = s;
	glProjection[2][0] = x0;
	glProjection[3][0] = 0;

	glProjection[0][1] = 0;
	glProjection[1][1] = fy;
	glProjection[2][1] = y0;
	glProjection[3][1] = 0;

	glProjection[0][2] = 0;
	glProjection[1][2] = 0;
	glProjection[2][2] = -(f + n) / (f - n);
	glProjection[3][2] = (-2.0 * n * f) / (f - n);

	glProjection[0][3] = 0;
	glProjection[1][3] = 0;
	glProjection[2][3] = -1;
	glProjection[3][3] = 0;
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
