#include "CameraProjectionMat.h"
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
using namespace defaulttype;

defaulttype::Vector2 CameraProjectionMat::get2DFrom3DPosition(const Vector3& p)
{
	const Mat3x4d& P = d_projection.getValue();

	double rx = P[0][0] * p[0] + P[0][1] * p[1] + P[0][2] * p[2] + P[0][3];
	double ry = P[1][0] * p[0] + P[1][1] * p[1] + P[1][2] * p[2] + P[1][3];
	double rz = P[2][0] * p[0] + P[2][1] * p[1] + P[2][2] * p[2] + P[2][3];

	return Vector2(rx, ry) * 1.0 / rz;
}

void CameraProjectionMat::remapVectors(const helper::vector<Vector3>& inpos3d,
																			 const helper::vector<Vector2>& inpos2d,
																			 std::vector<cv::Point3f>& outpos3d,
																			 std::vector<cv::Point2f>& outpos2d)
{
	if (inpos3d.empty() || inpos2d.empty()) return;

	if (d_remapdist.getValue() != 0)
	{
		helper::vector<Vector2> proj2d;

		for (unsigned i = 0; i < inpos3d.size(); i++)
		{
			proj2d.push_back(get2DFrom3DPosition(inpos3d[i]));
		}

		for (unsigned i = 0; i < inpos2d.size(); i++)
		{
			Vector2 p = inpos2d[i];

			int min = 0;
			for (unsigned j = 1; j < proj2d.size(); j++)
			{
				if ((p - proj2d[j]).norm() < (p - proj2d[min]).norm()) min = j;
			}

			if ((p - proj2d[min]).norm() < d_remapdist.getValue())
			{
				outpos3d.push_back(
						cv::Point3f(inpos3d[min][0], inpos3d[min][1], inpos3d[min][2]));
				outpos2d.push_back(cv::Point2f(inpos2d[i][0], inpos2d[i][1]));
			}
		}
	}
	else if (d_remapInd.getValue().size())
	{
		for (unsigned i = 0; i < d_remapInd.getValue().size(); i++)
		{
			const Vec2i& map = d_remapInd.getValue()[i];
			if (map[0] >= (int)inpos3d.size()) continue;
			if (map[1] >= (int)inpos2d.size()) continue;

			int id3d = map[0];
			int id2d = map[1];

			outpos3d.push_back(
					cv::Point3f(inpos3d[id3d][0], inpos3d[id3d][1], inpos3d[id3d][2]));
			outpos2d.push_back(cv::Point2f(inpos2d[id2d][0], inpos2d[id2d][1]));
		}
	}
	else
	{
		unsigned nbpts = std::min(inpos3d.size(), inpos2d.size());

		for (unsigned i = 0; i < nbpts; i++)
		{
			cv::Point3f pos3d(inpos3d[i][0], inpos3d[i][1], inpos3d[i][2]);
			outpos3d.push_back(pos3d);
		}

		for (unsigned i = 0; i < nbpts; i++)
		{
			cv::Point2f pos2d(inpos2d[i][0], inpos2d[i][1]);
			outpos2d.push_back(pos2d);
		}
	}
}

void CameraProjectionMat::setCamera(unsigned w, unsigned h)
{
	const Mat4x4d& mp = d_projectionMatrixOpenGL.getValue();
	const Mat4x4d& mm = d_modelViewMatrixOpenGL.getValue();

	double fx = mp[0][0];
	double s = mp[1][0];
	double x0 = mp[2][0];
	double fy = mp[1][1];
	double y0 = mp[2][1];

	Mat3x3d R;
	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 3; i++)
		{
			R[j][i] = mm[j][i];
		}
	}

	Vec2f oglCenter(0.0, 0.0);
	Mat3x3d K;
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

	Quat Orig =
			camera_ori * Quat(Vector3(0, 0, 1), M_PI) * Quat(Vector3(0, 1, 0), M_PI);
	Mat3x3d Rq;
	Orig.toMatrix(Rq);

	Mat3x3d C = K * Rq.transposed();
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

	d_projection.setValue(M);

	Rigid cpos;

	cpos.getCenter() = camera_pos;
	cpos.getOrientation() = camera_ori;

	d_cameraPos.setValue(cpos);

	//    std::cout << "pos " << d_cameraPos.getValue() << std::endl;

	m_intrinsicParameters[0] = fx;
	m_intrinsicParameters[1] = fy;
	m_intrinsicParameters[2] = s;
	m_intrinsicParameters[3] = x0;
	m_intrinsicParameters[4] = y0;

	m_cameraPos.getCenter() = camera_pos;
	m_cameraPos.getOrientation() = camera_ori;

	sout << "projectionMatrixOpenGL=\"" << d_projectionMatrixOpenGL.getValue()
			 << "\"" << sendl;
	sout << "modelViewMatrixOpenGL=\"" << d_modelViewMatrixOpenGL.getValue()
			 << "\"" << sendl;
	sout << "projectionMatrix=\"" << d_projection.getValue() << "\"" << sendl;
}

void CameraProjectionMat::setCamera(Vector3 camera_pos, Quat camera_ori,
																		double fx, double fy, double s, double x0,
																		double y0, unsigned w, unsigned h)
{
	double n = d_zNear.getValue();
	double f = d_zFar.getValue();  // these values are used only for rendering not
	// for calibration

	Mat4x4d MP;

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

	d_projectionMatrixOpenGL.setValue(MP);

	Mat4x4d MM;

	Mat3x3d R;
	camera_ori.toMatrix(R);
	Mat3x3d iR = R.transposed();

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

	d_modelViewMatrixOpenGL.setValue(MM);

	setCamera(w, h);
}

void CameraProjectionMat::setCamera(const Mat3x4d& M, unsigned w, unsigned h)
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
	double y0 =
			(-1.0 * h + 2.0 * cvmGet(cvK, 1, 2) /*+ 2.0*d_oglCenter.getValue()[1]*/) /
			h;

	Vector3 camera_pos =
			Vector3(cvmGet(cvT, 0, 0), cvmGet(cvT, 1, 0), cvmGet(cvT, 2, 0)) * 1.0 /
			cvmGet(cvT, 3, 0);

	Mat3x3d R;
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

	setCamera(camera_pos, camera_ori, fx, fy, s, x0, y0, w, h);
}

bool CameraProjectionMat::poseEstimation(const helper::vector<Vector3>& p3d,
																				 const helper::vector<Vector2>& p2d1,
																				 const helper::vector<Vector2>& p2d2,
																				 unsigned w, unsigned h, Mat3x4d& M)
{
	std::vector<cv::Point3f> pos3d;
	std::vector<cv::Point2f> pos2d1;
	std::vector<cv::Point2f> pos2d2;
	std::vector<std::vector<cv::Point2f> > pos2D;
	std::vector<std::vector<cv::Point3f> > pos3D;
	remapVectors(p3d, p2d1, pos3d, pos2d1);
	remapVectors(p3d, p2d2, pos3d, pos2d2);

	pos2D.push_back(pos2d1);
	pos2D.push_back(pos2d2);
	pos3D.push_back(pos3d);
	pos3D.push_back(pos3d);
	unsigned nbpts = pos3d.size();
	if (nbpts == 0) return false;

	//	// camMatrix based on img size
	//	int max_d = std::max(w, h);
	cv::Mat camMatrix =
			(cv::Mat_<double>(3, 3) << 2000, 0, w / 2.0, 0, 2000, h / 2.0, 0, 0, 1.0);

	//	std::vector<cv::Mat> rvecs, tvecs;
	//	//    cv::solvePnP(pts3d, pts2d, camMatrix, cv::Mat(1,4,CV_64F,0.0),
	// rvec,
	//	//    tvec, false, CV_ITERATIVE );
	//	cv::Mat dc;
	//	cv::calibrateCamera(pos3d, pos2D, cv::Size(w, h), camMatrix, dc, rvecs,
	// tvecs);

	std::vector<std::vector<cv::Point2f> > imagePoints;
	imagePoints.push_back(pos2d1);
	imagePoints.push_back(pos2d2);
	std::vector<std::vector<cv::Point3f> > objectPoints;
	objectPoints.push_back(pos3d);
	objectPoints.push_back(pos3d);
	//			cv::Mat camMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	//			std::cout << imagePoints.size() << " == " <<
	//objectPoints.size()
	//<< std::endl;

	objectPoints[0].resize(objectPoints[0].size() / 2);
	objectPoints[1].resize(objectPoints[1].size() / 2);

	std::cout << imagePoints[0].size() << " == " << objectPoints[0].size()
						<< std::endl;
	std::cout << imagePoints[1].size() << " == " << objectPoints[1].size()
						<< std::endl;
	cv::calibrateCamera(
			objectPoints, imagePoints, cv::Size(w, h), camMatrix, distCoeffs, rvecs,
			tvecs, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT |
								 CV_CALIB_FIX_ASPECT_RATIO);

	// get 3d rot mat
	cv::Mat rotM(3, 3, CV_64F);
	cv::Rodrigues(rvecs[0], rotM);

	// push tvec to transposed Mat
	cv::Mat rotMT = rotM.t();
	rotMT.push_back(tvecs[0].reshape(1, 1));

	// transpose back, and multiply
	cv::Mat Proj = camMatrix * rotMT.t();

	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 4; i++)
		{
			M[j][i] = Proj.at<double>(j, i);
		}
	}

	return true;
}

void CameraProjectionMat::update()
{
	Mat3x4d M;
	if (poseEstimation(d_pts3d.getValue(), d_pts2d1.getValue(),
										 d_pts2d2.getValue(), d_w.getValue(), d_h.getValue(), M))
		setCamera(M, d_w.getValue(), d_h.getValue());
}
}
}
}
