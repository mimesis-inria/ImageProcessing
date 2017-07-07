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

#ifndef SOFA_OR_PROCESSOR_CALIBRATESTEREO_H
#define SOFA_OR_PROCESSOR_CALIBRATESTEREO_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "camera/common/StereoSettings.h"

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
class CalibrateStereo : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibrateStereo, StereoSettings,
																							sofa::BaseLink::FLAG_STOREPATH |
																									sofa::BaseLink::FLAG_STRONGLINK>
			Settings;

 public:
	SOFA_CLASS(CalibrateStereo, common::ImplicitDataEngine);

	CalibrateStereo()
			: l_cam(initLink("cam",
												"link to the CameraSettings component containing and "
												"maintaining the reference camera's parameters")),
				d_imagePoints1(
						initData(&d_imagePoints1, "imagePoints1",
										 "a vector of vectors of the projections of the 3D "
										 "object's points in the reference camera's image. "
										 "imagePoints1.size() and "
										 "objectPoints.size() and imagePoints1[i].size() must "
										 "be equal to objectPoints1[i].size() for each i")),
				d_imagePoints2(
						initData(&d_imagePoints2, "imagePoints2",
										 "a vector of vectors of the projections of the 3D "
										 "object's points in the second camera's image. "
										 "imagePoints2.size() and "
										 "objectPoints.size() and imagePoints2[i].size() must "
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
						&d_calibFlags, 0x00101, "calibFlags",
						"One or a combination of the following flags:\n"
						"USE_INTRINSIC_GUESS (1): cameraMatrix contains "
						"valid initial values of fx, fy, cx, cy that are "
						"optimized further.\n"
						"FIX_ASPECT_RATIO (2): preserves the fx/fy ratio\n"
						"FIX_PRINCIPAL_POINT (4): The principal point won't "
						"change during optimization\n"
						"ZERO_TANGENT_DIST (8): Tangential distortion is set to 0"))
	{
	}

	~CalibrateStereo() {}
	void init()
	{
		addInput(&d_imagePoints1);
		addInput(&d_imagePoints2);
		addInput(&d_objectPoints);
		addInput(&d_imgSize);

		if (!l_cam.get())
			msg_error(getName() + "::init()") << "Error: No Stereo camera settings link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";
		update();
	}

	void update();
	void calibrate();

	Settings l_cam;

	// INPUTS
	sofa::Data<sofa::helper::SVector<sofa::helper::SVector<sofa::defaulttype::Vector2> > > d_imagePoints1;
	sofa::Data<sofa::helper::SVector<sofa::helper::SVector<sofa::defaulttype::Vector2> > > d_imagePoints2;
	sofa::Data<sofa::helper::SVector<sofa::helper::SVector<sofa::defaulttype::Vector3> > > d_objectPoints;
	sofa::Data<sofa::defaulttype::Vec2i> d_imgSize;

	// OPTIONAL INPUTS
	sofa::Data<int> d_calibFlags;
};

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CALIBRATESTEREO_H
