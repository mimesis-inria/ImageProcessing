#ifndef SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
#define SOFA_OR_PROCESSOR_IMAGERECTIFIER_H

#include "CameraSettings.h"
#include "ProcessOR/common/ImageFilter.h"
#include "SofaORCommon/cvMatUtils.h"

#include <opencv2/imgproc.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
/**
 * @brief The ImageRectifier class
 *
 * Rectifies a given image frame using the linked CameraSettings parameters
 */
class ImageRectifier : public ImageFilter
{
	typedef sofa::core::objectmodel::SingleLink<
			ImageRectifier, CameraSettings,
			sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
  SOFA_CLASS(ImageRectifier, ImageFilter);

  ImageRectifier()
			: l_cam(initLink("cam",
											 "link to CameraSettings component containing and "
											 "maintaining the camera's parameters"))
  {
  }

  void init()
  {
		if (!l_cam.get())
			msg_error(getName() + "::init()") << "Error: No camera link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";
    ImageFilter::init();
  }
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
		if (in.empty() || l_cam->getDistortionCoefficients().empty()) return;
    cv::Mat_<double> cam;
		common::matrix::sofaMat2cvMat(l_cam->getIntrinsicCameraMatrix(), cam);
		cv::undistort(in, out, cam, l_cam->getDistortionCoefficients());
  }

	CamSettings l_cam;  ///< linked CameraSettings component
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
		sofa::core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
