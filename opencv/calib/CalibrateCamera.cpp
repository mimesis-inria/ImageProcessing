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
	std::vector<std::vector<cv::Point2f> > imgPts;
	std::vector<std::vector<cv::Point3f> > objPts;

	for (auto pts : d_objectPoints.getValue())
	{
		std::vector<cv::Point3f> objPoints;
		for (auto pt : pts)
			objPoints.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
		objPts.push_back(objPoints);
	}
	for (auto pts : d_imagePoints.getValue())
	{
		std::vector<cv::Point2f> imgPoints;
		for (auto pt : pts) imgPoints.push_back(cv::Point2f(pt.x(), pt.y()));
		imgPts.push_back(imgPoints);
	}

	//	cv::Mat_<double> camMatrix;
	cv::Mat camMatrix =
			(cv::Mat_<double>(3, 3) << 2000, 0, d_imgSize.getValue().x() / 2.0, 0,
			 2000, d_imgSize.getValue().y() / 2.0, 0, 0, 1.0);
	cv::Mat dc;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	try
	{
		if (d_K.isSet()) common::matrix::sofaMat2cvMat(d_K.getValue(), camMatrix);
		if (d_distCoefs.isSet())
			common::matrix::sofaVector2cvMat(d_distCoefs.getValue(), dc);

		cv::calibrateCamera(objPts, imgPts, cv::Size(d_imgSize.getValue().x(),
																								 d_imgSize.getValue().y()),
												camMatrix, dc, rvecs, tvecs, d_calibFlags.getValue());
		std::cout << camMatrix << "    " << dc << std::endl;
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

	if (!d_preserveExtrinsics.getValue())
	{
		const defaulttype::Matrix3& R = d_rvecs.getValue().back();
		const defaulttype::Vector3& t = d_tvecs.getValue().back();

		l_cam->setRotationMatrix(R, false);
		l_cam->setTranslationVector(t, false);
	}

	l_cam->setIntrinsicCameraMatrix(K, true);
	l_cam->setDistortionCoefficients(m_distCoefs);

}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
