#include "ProjectPoints.h"

namespace sofa
{
namespace OR
{
namespace processor
{

SOFA_DECL_CLASS(ProjectPoints)

int ProjectPointsClass =
		core::RegisterObject("Component projecting points from 2D to 3D & vice versa using a linked CameraSettings")
				.add<ProjectPoints>();

} // namespace processor
} // namespace OR
} // namespace sofa
