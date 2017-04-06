#include "CalibrateCamera.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(CalibrateCamera)

int CalibrateCameraClass =
		core::RegisterObject(
				"Component calibrating a monoscopic camera from a set of 2D and 3D "
				"correspondances. computed matrices are set in the linked "
				"CameraSettings, while the multiple translation vectors are rotation "
				"matrices estimated for each set of 2D/3D correspondences are made "
				"available for further processing")
				.add<CalibrateCamera>();


void CalibrateCamera::calibrate()
{
	std::vector<std::vector<cv::Point2d> > imgPts;
	std::vector<std::vector<cv::Point3d> > objPts;

	for (auto pts : d_objectPoints.getValue())
	{
		std::vector<cv::Point3d> objPoints;
		for (auto pt : pts)
			objPoints.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
		objPts.push_back(objPoints);
	}
	for (auto pts : d_imagePoints.getValue())
	{
		std::vector<cv::Point2d> imgPoints;
		for (auto pt : pts) imgPoints.push_back(cv::Point2f(pt.x(), pt.y()));
		imgPts.push_back(imgPoints);
	}

	cv::Mat_<double> camMatrix;
	cv::Mat_<double> dc;
	std::vector<cv::Mat_<double> > rvecs;
	std::vector<cv::Mat_<double> > tvecs;
	try
	{
		common::matrix::sofaMat2cvMat(d_K.getValue(), camMatrix);
		common::matrix::sofaVector2cvMat(d_distCoefs.getValue(), dc);

		cv::calibrateCamera(objPts, imgPts, cv::Size(d_imgSize.getValue().x(),
																								 d_imgSize.getValue().y()),
												camMatrix, dc, rvecs, tvecs, d_calibFlags.getValue());
	}
	catch (cv::Exception& e)
	{
		msg_error(getName() + "::calibrate()") << e.what();
	}

	common::matrix::cvMat2sofaVector(dc, m_distCoefs);

	common::matrix::cvMat2sofaMat(camMatrix, m_K);
	helper::vector<defaulttype::Matrix3>& rotations = *d_rvecs.beginEdit();
	helper::vector<defaulttype::Vector3>& translations = *d_tvecs.beginEdit();
	for (unsigned i = 0; i < rvecs.size(); ++i)
	{
		defaulttype::Matrix3 rot;
		defaulttype::Vector3 tr;
		common::matrix::cvMat2sofaMat(rvecs[i], rot);
		common::matrix::cvMat2sofaVector(tvecs[i], tr);
		rotations.push_back(rot);
		translations.push_back(tr);
	}
	d_rvecs.endEdit();
	d_tvecs.endEdit();
}

void CalibrateCamera::update()
{
	calibrate();

	const defaulttype::Matrix3& K = m_K;

	if (d_preserveExtrinsics.getValue())
	{
		l_cam->setIntrinsicCameraMatrix(K);
		l_cam->setDistortionCoefficients(m_distCoefs);
	}
	else
	{
		const defaulttype::Matrix3& R = d_rvecs.getValue().back();
		const defaulttype::Vector3& t = d_tvecs.getValue().back();

		defaulttype::Mat3x4d P =
				defaulttype::Mat3x4d(defaulttype::Vec4d(R[0][0], R[0][1], R[0][2], t[0]),
				defaulttype::Vec4d(R[1][0], R[1][1], R[1][2], t[1]),
				defaulttype::Vec4d(R[2][0], R[2][1], R[2][2], t[2]));
		P = K * P;
		l_cam->setProjectionMatrix(P);
		l_cam->setDistortionCoefficients(m_distCoefs);
	}
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
