#ifndef SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
#define SOFA_OR_PROCESSOR_IMAGERECTIFIER_H

#include "CameraSettings.h"
#include "ProcessOR/common/ImageFilter.h"
#include "SofaORCommon/cvMatUtils.h"

#include <opencv2/imgproc.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class ImageRectifier : public ImageFilter
{
	typedef sofa::core::objectmodel::SingleLink<ImageRectifier, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
  SOFA_CLASS(ImageRectifier, ImageFilter);

  Data<defaulttype::Matrix3> d_projMat;
  Data<helper::vector<double> > d_distCoefs;

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

	CamSettings l_cam;
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
    core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
