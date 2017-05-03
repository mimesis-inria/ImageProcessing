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

		std::cout << cv::calibrateCamera(
										 objPts, imgPts, cv::Size(d_imgSize.getValue().x(),
																							d_imgSize.getValue().y()),
										 camMatrix, dc, rvecs, tvecs, d_calibFlags.getValue())
							<< std::endl;
		//		std::cout << camMatrix << "    " << dc << std::endl;
	}
	catch (cv::Exception& e)
	{
		msg_error(getName() + "::calibrate()") << e.what();
	}

	for (auto m : tvecs) std::cout << m << std::endl;

	common::matrix::cvMat2sofaVector(dc, m_distCoefs);

	common::matrix::cvMat2sofaMat(camMatrix, m_K);
	//	helper::vector<defaulttype::Matrix3>& rotations = *d_rvecs.beginEdit();
	helper::vector<defaulttype::Mat3x4d>& RTs = *d_Rts.beginEdit();
	//	helper::vector<defaulttype::Vector3>& translations =
	//*d_tvecs.beginEdit();
	for (unsigned i = 0; i < rvecs.size(); ++i)
	{
		// get 3d rot mat
		cv::Mat rotM(3, 3, CV_64F);
		cv::Rodrigues(rvecs[i], rotM);

		// push tvec to transposed Mat
		// tvec is ALREADY the 3rd column of a 3x4 proj matrix... so just append it
		// to the Rotation matrix
		cv::Mat rotMT = rotM.t();
		rotMT.push_back(tvecs[0].reshape(1, 1));
		cv::Mat P = camMatrix * rotMT.t();

		defaulttype::Mat3x4d ProjMat;
		common::matrix::cvMat2sofaMat(P, ProjMat);

		std::cout << "rotation matrices retrieved in sofa" << std::endl;
		RTs.push_back(ProjMat);
	}

	//	d_rvecs.endEdit();
	//	d_tvecs.endEdit();
	d_Rts.endEdit();
}

void CalibrateCamera::update()
{
	calibrate();

	const defaulttype::Matrix3& K = m_K;

	if (!d_preserveExtrinsics.getValue())
		l_cam->setProjectionMatrix(d_Rts.getValue().back());
	else
	{
		l_cam->setIntrinsicCameraMatrix(K, true);
	}
	l_cam->setDistortionCoefficients(m_distCoefs);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
