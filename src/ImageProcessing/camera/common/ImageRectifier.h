#ifndef SOFACV_CAM_IMAGERECTIFIER_H
#define SOFACV_CAM_IMAGERECTIFIER_H

#include <SofaCV/SofaCV.h>
#include "CameraSettings.h"

#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace cam
{
/**
 * @brief The ImageRectifier class
 *
 * Rectifies a given image frame using the linked CameraSettings parameters
 */
class SOFA_IMAGEPROCESSING_API ImageRectifier : public ImageFilter
{
  typedef sofa::core::objectmodel::SingleLink<
      ImageRectifier, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(ImageRectifier, ImageFilter);

  ImageRectifier();

  void init();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

  CamSettings l_cam;  ///< linked CameraSettings component
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
    sofa::core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_IMAGERECTIFIER_H
