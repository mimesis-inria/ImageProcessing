#ifndef SOFA_OR_PROCESSOR_CALIBRATESTEREO_H
#define SOFA_OR_PROCESSOR_CALIBRATESTEREO_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "CameraSettings.h"

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
	typedef sofa::core::objectmodel::SingleLink<CalibrateStereo, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(CalibrateStereo, common::ImplicitDataEngine);

	CalibrateStereo()
			: l_cam1(initLink("cam1",
												"link to the CameraSettings component containing and "
												"maintaining the reference camera's parameters")),
				l_cam2(initLink("cam2",
												"link to the CameraSettings component containing and "
												"maintaining the second camera's parameters")),
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
				d_imgSize(initData(&d_imgSize, "imgSize",
													 "size in px of the image (used to initialize the "
													 "intrinsic camera matrix")),
				d_calibFlags(initData(
						&d_calibFlags, 0, "calibFlags",
						"One or a combination of the following flags:\n"
						"USE_INTRINSIC_GUESS (1): cameraMatrix contains "
						"valid initial values of fx, fy, cx, cy that are "
						"optimized further.\n"
						"FIX_ASPECT_RATIO (2): preserves the fx/fy ratio\n"
						"FIX_PRINCIPAL_POINT (4): The principal point won't "
						"change during optimization\n"
						"ZERO_TANGENT_DIST (8): Tangential distortion is set to 0")),
				d_K1(initData(&d_K1, "K1",
											"[Optional] reference camera's intrinsic params used to "
											"provide initial guesses (depending on "
											"used calibFlags)")),
				d_K2(initData(&d_K2, "K2",
											"[Optional] reference camera's intrinsic params used to "
											"provide initial guesses (depending on "
											"used calibFlags)")),
				d_distCoefs1(initData(&d_distCoefs1, "distCoefs1",
														 "[Optional] ref camera's distortion coefficients initial guess "
														 "(check calibFlags)")),
				d_distCoefs2(initData(&d_distCoefs2, "distCoefs2",
														 "[Optional] 2nd camera's distortion coefficients initial guess "
														 "(check calibFlags)")),
				d_R(initData(&d_R, "R",
												 "rotation matrix for each 2D "
												 "projections provided by imagePoints (expressed in "
												 "the object's coordinate space")),
				d_t(initData(&d_t, "tvecs_out",
												 "translation vectors for each 2D "
												 "projections provided by imagePoints (expressed in "
												 "the object's coordinate space"))
	{
	}

	~CalibrateStereo() {}
	void init()
	{
		addInput(&d_imagePoints);
		addInput(&d_objectPoints);
		addInput(&d_imgSize);
		addOutput(&d_K);
		addOutput(&d_distCoefs);

		addOutput(&d_rvecs);
		addOutput(&d_tvecs);

		update();
	}

	void update();
	void calibrate();

	CamSettings l_cam;

	// INPUTS
	Data<helper::SVector<helper::SVector<defaulttype::Vector2> > > d_imagePoints;
	Data<helper::SVector<helper::SVector<defaulttype::Vector3> > > d_objectPoints;
	Data<defaulttype::Vec2i> d_imgSize;

	// OPTIONAL INPUTS
	Data<int> d_calibFlags;
	Data<defaulttype::Matrix3> d_K;
	Data<helper::vector<double> > d_distCoefs;

	// OUTPUTS
	Data<helper::vector<defaulttype::Matrix3> > d_rvecs;
	Data<helper::vector<defaulttype::Vector3> > d_tvecs;

 private:
	defaulttype::Matrix3 m_K;
	helper::vector<double> m_distCoefs;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CALIBRATESTEREO_H
