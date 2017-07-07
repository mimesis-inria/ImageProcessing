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

#ifndef SOFA_OR_PROCESSOR_CALIBRATECAMERA_H
#define SOFA_OR_PROCESSOR_CALIBRATECAMERA_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "camera/common/CameraSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
class CalibrateCamera : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibrateCamera, CameraSettings,
																							sofa::BaseLink::FLAG_STOREPATH |
																									sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(CalibrateCamera, common::ImplicitDataEngine);

	CalibrateCamera()
			: l_cam(initLink("cam",
											 "link to CameraSettings component containing and "
											 "maintaining the camera's parameters")),
				d_imagePoints(
						initData(&d_imagePoints, "imagePoints",
										 "a vector of vectors of the projections of the 3D "
										 "object's points in the image. imagePoints.size() and "
										 "objectPoints.size() and imagePoints[i].size() must "
										 "be equal to objectPoints[i].size() for each i")),
				d_objectPoints(
						initData(&d_objectPoints, "objectPoints",
										 "a vector of vectors of calibration pattern "
										 "points in the calibration pattern's coordinate "
										 "space")),
				d_imgSize(initData(&d_imgSize, "imageSize",
													 "size in px of the image (used to initialize the "
													 "intrinsic camera matrix")),
				d_calibFlags(initData(
						&d_calibFlags, 1, "calibFlags",
						"One or a combination of the following flags:\n"
						"USE_INTRINSIC_GUESS (1): cameraMatrix contains "
						"valid initial values of fx, fy, cx, cy that are "
						"optimized further.\n"
						"FIX_ASPECT_RATIO (2): preserves the fx/fy ratio\n"
						"FIX_PRINCIPAL_POINT (4): The principal point won't "
						"change during optimization\n"
						"ZERO_TANGENT_DIST (8): Tangential distortion is set to 0")),
				d_K(initData(&d_K, "K",
										 "Required for non-planar calbiration rigs, Optional "
										 "otherwise. used to provide initial guesses (depending on "
										 "used calibFlags)")),
				d_distCoefs(initData(&d_distCoefs, "distCoefs",
														 "[Optional] distortion coefficients initial guess "
														 "(check calibFlags)")),
				d_preserveExtrinsics(
						initData(&d_preserveExtrinsics, false, "keepExtrinsics",
										 "if true, only intrinsics are updated. Otherwise, "
										 "extrinsics are set to the last frame's pose estimation"))
	{
	}

	~CalibrateCamera() {}
	void init()
	{
		addInput(&d_imagePoints);
		addInput(&d_objectPoints);
		addInput(&d_imgSize);
		addOutput(&d_K);
		addOutput(&d_distCoefs);

		if (!l_cam.get())
			msg_error(getName() + "::init()") << "Error: No camera link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";
		update();
	}

	void update();
	void calibrate();

	CamSettings l_cam;

	// INPUTS
	sofa::Data<sofa::helper::SVector<sofa::helper::SVector<sofa::defaulttype::Vector2> > > d_imagePoints;
	sofa::Data<sofa::helper::SVector<sofa::helper::SVector<sofa::defaulttype::Vector3> > > d_objectPoints;
	sofa::Data<sofa::defaulttype::Vec2i> d_imgSize;

	// OPTIONAL INPUTS
	sofa::Data<int> d_calibFlags;
	sofa::Data<sofa::defaulttype::Matrix3> d_K;
	sofa::Data<sofa::helper::vector<double> > d_distCoefs;

	// OUTPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Mat3x4d> > d_Rts;

	sofa::Data<bool> d_preserveExtrinsics;

 private:
	sofa::defaulttype::Matrix3 m_K;
	sofa::helper::vector<double> m_distCoefs;
};

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CALIBRATECAMERA_H
