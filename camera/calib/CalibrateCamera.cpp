/******************************************************************************
*       SOFAOR, SOFA plugin for the Operating Room, development version       *
*                        (c) 2017 INRIA, MIMESIS Team                         *
*                                                                             *
* This program is a free software; you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 1.0 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Bruno Marques and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact-mimesis@inria.fr                               *
******************************************************************************/

#include "CalibrateCamera.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
SOFA_DECL_CLASS(CalibrateCamera)

int CalibrateCameraClass =
		sofa::core::RegisterObject(
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
	}
	catch (cv::Exception& e)
	{
		msg_error(getName() + "::calibrate()") << e.what();
	}

	common::matrix::cvMat2sofaVector(dc, m_distCoefs);

	common::matrix::cvMat2sofaMat(camMatrix, m_K);
	sofa::helper::vector<sofa::defaulttype::Mat3x4d>& RTs = *d_Rts.beginEdit();
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

		sofa::defaulttype::Mat3x4d ProjMat;
		common::matrix::cvMat2sofaMat(P, ProjMat);

		RTs.push_back(ProjMat);
	}

	//	d_rvecs.endEdit();
	//	d_tvecs.endEdit();
	d_Rts.endEdit();
}

void CalibrateCamera::update()
{
	calibrate();

	const sofa::defaulttype::Matrix3& K = m_K;

	if (!d_preserveExtrinsics.getValue())
		l_cam->setProjectionMatrix(d_Rts.getValue().back());
	else
	{
		l_cam->setIntrinsicCameraMatrix(K, true);
	}
	l_cam->setDistortionCoefficients(m_distCoefs);
}

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
