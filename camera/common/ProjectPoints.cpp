#include "ProjectPoints.h"

namespace sofaor
{
namespace processor
{
namespace cam
{

SOFA_DECL_CLASS(ProjectPoints)

int ProjectPointsClass =
		sofa::core::RegisterObject("Component projecting points from 2D to 3D & vice versa using a linked CameraSettings")
				.add<ProjectPoints>();

} // namespace cam
} // namespace processor
} // namespace sofaor

