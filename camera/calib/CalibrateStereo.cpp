#include "CalibrateStereo.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(CalibrateStereo)

int CalibrateStereoClass =
		core::RegisterObject(
				"Component calibrating a Stereoscopic camera from two sets of 2D and "
				"3D correspondances respectively referring to the reference camera and "
				"the secondary camera. Computed Extrinsics are set in the linked "
				"StereoSettings")
				.add<CalibrateStereo>();

void CalibrateStereo::calibrate()
{
	std::vector<std::vector<cv::Point3f> > objectPoints;
	std::vector<std::vector<cv::Point2f> > imagePoints1;
	std::vector<std::vector<cv::Point2f> > imagePoints2;

	if (d_imagePoints1.getValue().size() != d_imagePoints2.getValue().size() ||
			d_imagePoints1.getValue().size() < 1 ||
			d_imagePoints1.getValue()[0].size() !=
					d_imagePoints2.getValue()[0].size() ||
			d_imagePoints1.getValue()[0].size() !=
					d_objectPoints.getValue()[0].size())
		msg_error(getName() + "::calibrate()") << "Error: Vector size should be "
																							"the same for imagePoints1, "
																							"imagePoint2 and objectPoints";

	for (auto pts : d_objectPoints.getValue())
	{
		std::vector<cv::Point3f> objPts;
		for (auto pt : pts)
		{
			objPts.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
		}
		objectPoints.push_back(objPts);
	}
	for (auto pts : d_imagePoints1.getValue())
	{
		std::vector<cv::Point2f> imgPts;
		for (auto pt : pts)
		{
			imgPts.push_back(cv::Point2f(pt.x(), pt.y()));
		}
		imagePoints1.push_back(imgPts);
	}
	for (auto pts : d_imagePoints2.getValue())
	{
		std::vector<cv::Point2f> imgPts;
		for (auto pt : pts)
		{
			imgPts.push_back(cv::Point2f(pt.x(), pt.y()));
		}
		imagePoints2.push_back(imgPts);
	}

	cv::Mat_<double> distCoeffs1;
	cv::Mat_<double> distCoeffs2;
	cv::Mat Rmat;
	cv::Mat Tvec;
	cv::Mat E;
	cv::Mat F;

	cv::Mat_<double> cam1, cam2;
	common::matrix::sofaMat2cvMat(l_cam->getCamera1().getIntrinsicCameraMatrix(),
																cam1);
	common::matrix::sofaMat2cvMat(l_cam->getCamera2().getIntrinsicCameraMatrix(),
																cam2);

	common::matrix::sofaVector2cvMat(
			l_cam->getCamera1().getDistortionCoefficients(), distCoeffs1);
	common::matrix::sofaVector2cvMat(
			l_cam->getCamera2().getDistortionCoefficients(), distCoeffs2);

	std::cout << "reprojectionError: "
						<< cv::stereoCalibrate(
									 objectPoints, imagePoints1, imagePoints2, cam1, distCoeffs1,
									 cam2, distCoeffs2,
									 cv::Size(d_imgSize.getValue().x(), d_imgSize.getValue().y()),
									 Rmat, Tvec, E, F,
									 cv::CALIB_FIX_INTRINSIC | cv::CALIB_USE_INTRINSIC_GUESS)
						<< std::endl;

//	l_cam->setRotationMatrix(defaulttype::Matrix3((double*)Rmat.ptr()));
//	l_cam->setTranslationVector(defaulttype::Vector3((double*)Tvec.ptr()));
	l_cam->setEssentialMatrix(defaulttype::Matrix3((double*)E.ptr()));
	l_cam->setFundamentalMatrix(defaulttype::Matrix3((double*)F.ptr()));
}

void CalibrateStereo::update()
{
	if (d_imagePoints1.isSet() && d_imagePoints2.isSet() &&
			d_objectPoints.isSet())
		calibrate();
	else
	{
		std::cout << "computing stereo params from cameras" << std::endl;
		l_cam->recomputeFromCameras();
	}
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
