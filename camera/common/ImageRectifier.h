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

#ifndef SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
#define SOFA_OR_PROCESSOR_IMAGERECTIFIER_H

#include "CameraSettings.h"
#include "ProcessOR/common/ImageFilter.h"
#include "SofaORCommon/cvMatUtils.h"

#include <opencv2/imgproc.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
/**
 * @brief The ImageRectifier class
 *
 * Rectifies a given image frame using the linked CameraSettings parameters
 */
class ImageRectifier : public ImageFilter
{
	typedef sofa::core::objectmodel::SingleLink<
			ImageRectifier, CameraSettings,
			sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
  SOFA_CLASS(ImageRectifier, ImageFilter);

  ImageRectifier()
			: l_cam(initLink("cam",
											 "link to CameraSettings component containing and "
											 "maintaining the camera's parameters"))
  {
  }

  void init()
  {
		if (!l_cam.get())
			msg_error(getName() + "::init()") << "Error: No camera link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";
    ImageFilter::init();
  }
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
		if (in.empty() || l_cam->getDistortionCoefficients().empty()) return;
    cv::Mat_<double> cam;
		common::matrix::sofaMat2cvMat(l_cam->getIntrinsicCameraMatrix(), cam);
		cv::undistort(in, out, cam, l_cam->getDistortionCoefficients());
  }

	CamSettings l_cam;  ///< linked CameraSettings component
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
		sofa::core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
