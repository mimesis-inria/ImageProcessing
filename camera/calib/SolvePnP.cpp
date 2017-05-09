#include "SolvePnP.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(SolvePnP)

int SolvePnPClass =
		core::RegisterObject(
				"The Component estimates the object pose given a set of object points, "
				"their corresponding image projections, as well as the camera matrix "
				"and the distortion coefficients.")
				.add<SolvePnP>();

void SolvePnP::update()
{
	std::vector<cv::Point2d> imgPts;
	std::vector<cv::Point3d> objPts;

	for (auto pt : d_objectPoints.getValue())
		objPts.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
	for (auto pt : d_imagePoints.getValue())
		imgPts.push_back(cv::Point2f(pt.x(), pt.y()));

	cv::Mat_<double> camMatrix;
	cv::Mat_<double> dc;
	cv::Mat rvec, tvec;
	defaulttype::Vec2i imsize = d_imgSize.getValue();
	try
	{
		if (d_K.isSet())
			common::matrix::sofaMat2cvMat(d_K.getValue(), camMatrix);
		else if (l_cam->getIntrinsicCameraMatrix() ==
						 defaulttype::Matrix3::Identity())
			common::matrix::sofaMat2cvMat(l_cam->getIntrinsicCameraMatrix(),
																		camMatrix);
		else
		{
			int max_d;
			if (d_imgSize.isSet())
				imsize = d_imgSize.getValue();
			else
				imsize = l_cam->getImageSize();
			max_d = std::max(imsize.x(), imsize.y());
			camMatrix = (cv::Mat_<double>(3, 3) << max_d, 0, imsize.x() / 2.0, 0,
									 max_d, imsize.y() / 2.0, 0, 0, 1.0);
		}
		if (d_distCoefs.isSet())
			common::matrix::sofaVector2cvMat(d_distCoefs.getValue(), dc);
		else
			common::matrix::sofaVector2cvMat(l_cam->getDistortionCoefficients(), dc);

		cv::solvePnP(objPts, imgPts, camMatrix, dc, rvec, tvec, false, d_pnpFlags.getValue());
	}
	catch (cv::Exception& e)
	{
		msg_error(getName() + "::update()") << e.what();
		return;
	}

	defaulttype::Mat3x4d P;
	cv::Mat rotM(3, 3, CV_64F);
	cv::Rodrigues(rvec, rotM);

	// push tvec to transposed Mat
	cv::Mat rotMT = rotM.t();
	rotMT.push_back(tvec.reshape(1, 1));

	// transpose back, and multiply
	cv::Mat Proj = camMatrix * rotMT.t();

	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 4; i++)
		{
			P[j][i] = Proj.at<double>(j, i);
		}
	}

	helper::vector<double> distCoefs;
	common::matrix::cvMat2sofaVector(dc, distCoefs);

	if (l_cam->getImageSize() != imsize)
		l_cam->setImageSize(d_imgSize.getValue());
	l_cam->setProjectionMatrix(P);
	l_cam->setDistortionCoefficients(distCoefs);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
