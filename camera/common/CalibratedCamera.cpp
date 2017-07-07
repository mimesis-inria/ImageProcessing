#include "CalibratedCamera.h"

namespace sofaor
{
namespace processor
{
namespace cam
{
SOFA_DECL_CLASS(CalibratedCamera)

int CalibratedCameraClass =
		sofa::core::RegisterObject(
				"Component setting the CameraSettings to the camera")
				.add<CalibratedCamera>();

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
