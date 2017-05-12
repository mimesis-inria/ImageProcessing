#ifndef SOFA_OR_PROCESSOR_PROJECTPOINTS_H
#define SOFA_OR_PROCESSOR_PROJECTPOINTS_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "CameraSettings.h"

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class ProjectPoints : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<ProjectPoints, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

	typedef typename defaulttype::Vector3 Vector3;
	typedef typename defaulttype::Vector2 Vector2;

 public:
	SOFA_CLASS(ProjectPoints, common::ImplicitDataEngine);

	ProjectPoints()
			: l_cam(initLink("cam",
											 "link to CameraSettings component containing and "
											 "maintaining the camera's parameters")),
				d_2Dto3D(initData(&d_2Dto3D, true, "to3D", "if false, 3D to 2D")),
				d_depth(initData(&d_depth, -1.0f, "depth", "default is -1 (retrieves depth from fz in camSettings)")),
				d_Pts3D(initData(&d_Pts3D, "points3D", "3D points")),
				d_Pts2D(initData(&d_Pts2D, "points2D", "2D points"))
	{
	}

	~ProjectPoints() {}
	void init()
	{
		if (!l_cam.get())
			msg_error(getName() + "::init()") << "Error: No camera link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";
		if (d_2Dto3D.getValue())
		{
			addInput(&d_Pts2D);
			addOutput(&d_Pts3D);
		}
		else
		{
			addInput(&d_Pts3D);
			addOutput(&d_Pts2D);
		}
		update();
	}

	void update()
	{
		if (d_2Dto3D.getValue())
		{
			helper::vector<Vector3>& pts3d = *d_Pts3D.beginEdit();
			pts3d.clear();
			for (auto pt : d_Pts2D.getValue())
				pts3d.push_back(l_cam->get3DFrom2DPosition(pt, d_depth.getValue()));
		}
		else
		{
			helper::vector<Vector2>& pts2d = *d_Pts2D.beginEdit();
			pts2d.clear();
			for (auto pt : d_Pts3D.getValue())
				pts2d.push_back(l_cam->get2DFrom3DPosition(pt));
		}
	}

	CamSettings l_cam;
	Data<bool> d_2Dto3D;
	Data<float> d_depth;
	Data<helper::vector<Vector3> > d_Pts3D;
	Data<helper::vector<Vector2> > d_Pts2D;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_PROJECTPOINTS_H
