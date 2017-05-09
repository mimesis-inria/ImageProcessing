#include "CalibratedCamera.h"

namespace sofa
{
namespace OR
{
namespace processor
{

SOFA_DECL_CLASS(CalibratedCamera)

int CalibratedCameraClass =
		core::RegisterObject("Component setting the CameraSettings to the camera")
				.add<CalibratedCamera>();

} // namespace processor
} // namespace OR
} // namespace sofa
