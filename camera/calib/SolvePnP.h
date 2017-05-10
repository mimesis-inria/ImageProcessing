#ifndef SOFA_OR_PROCESSOR_SOLVEPNP_H
#define SOFA_OR_PROCESSOR_SOLVEPNP_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "camera/common/CameraSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class SolvePnP : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<SolvePnP, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(SolvePnP, common::ImplicitDataEngine);

	SolvePnP()
			: l_cam(initLink("cam",
											 "link to CameraSettings component containing and "
											 "maintaining the camera's parameters")),
				d_imagePoints(
						initData(&d_imagePoints, "imagePoints",
										 "a vector of 2D projections of the 3D "
										 "object's points in the image. imagePoints.size() and "
										 "objectPoints.size() must "
										 "be equal")),
				d_objectPoints(
						initData(&d_objectPoints, "objectPoints",
										 "a vector of calibration pattern "
										 "points in the calibration pattern's coordinate "
										 "space")),
				d_imgSize(initData(&d_imgSize, "imageSize",
													 "[Optional] size in px of the image. If not "
													 "provided, recovered from CameraSettings")),
				d_K(initData(&d_K, "K",
										 "[Optional] Intrinsic matrix. If not provided, intrinsics "
										 "will be recovered from CameraSettings, or approximated "
										 "from the imageSize")),
				d_distCoefs(
						initData(&d_distCoefs, "distCoefs",
										 "[Optional] distortion coefficients initial guess")),
				d_pnpFlags(initData(
						&d_pnpFlags, 4, "pnpFlags",
						"One or a combination of the following flags:\n"
						"ITERATIVE (1): Iterative method based on Levenberg-Marquardt "
						"optimization.\n"
						"P3P (2): Method based on the paper of X.S. Gao, X.-R. Hou, J. "
						"Tang, H.-F. Chang “Complete Solution Classification for the "
						"Perspective-Three-Point Problem”. In this case the function "
						"requires exactly four object and image points.\n"
						"EPNP (4): Method introduced by F.Moreno-Noguer, V.Lepetit and "
						"P.Fua in the paper “EPnP: Efficient Perspective-n-Point Camera "
						"Pose Estimation\n"))
	{
	}

	~SolvePnP() {}
	void init()
	{
		addInput(&d_imagePoints);
		addInput(&d_objectPoints);
		addInput(&d_imgSize);
		addOutput(&d_K);
		addOutput(&d_distCoefs);

		if (!(l_cam.get()))
			msg_error(this->getName() + "::init()") << "Error: No camera link set. "
																								 "Please use attribute 'cam' "
																								 "to define one";
		update();
	}

	void update();

	// Camera settings to update
	CamSettings l_cam;

	// INPUTS
	Data<helper::vector<defaulttype::Vector2> > d_imagePoints;
	Data<helper::vector<defaulttype::Vector3> > d_objectPoints;

	// OPTIONAL INPUT
	Data<defaulttype::Vec2i> d_imgSize;
	Data<defaulttype::Matrix3> d_K;
	Data<helper::vector<double> > d_distCoefs;
	Data<int> d_pnpFlags;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_SOLVEPNP_H
