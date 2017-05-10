#ifndef SOFA_OR_PROCESSOR_CALIBRATESTEREO_H
#define SOFA_OR_PROCESSOR_CALIBRATESTEREO_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "camera/common/StereoSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class CalibrateStereo : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibrateStereo, StereoSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
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
	Data<helper::SVector<helper::SVector<defaulttype::Vector2> > > d_imagePoints1;
	Data<helper::SVector<helper::SVector<defaulttype::Vector2> > > d_imagePoints2;
	Data<helper::SVector<helper::SVector<defaulttype::Vector3> > > d_objectPoints;
	Data<defaulttype::Vec2i> d_imgSize;

	// OPTIONAL INPUTS
	Data<int> d_calibFlags;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CALIBRATESTEREO_H
