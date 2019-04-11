#include "ImageRectifier.h"

namespace sofacv
{
namespace cam
{
ImageRectifier::ImageRectifier()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters"))
{
}

void ImageRectifier::init()
{
  if (!l_cam.get())
    msg_error(getName() + "::init()") << "Error: No camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";
  ImageFilter::init();
}

void ImageRectifier::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty() || l_cam->getDistortionCoefficients().empty()) return;
  cv::Mat_<double> cam;
  matrix::sofaMat2cvMat(l_cam->getIntrinsicCameraMatrix(), cam);
  cv::undistort(in, out, cam, l_cam->getDistortionCoefficients());
}
}  // namespace cam
}  // namespace sofacv
