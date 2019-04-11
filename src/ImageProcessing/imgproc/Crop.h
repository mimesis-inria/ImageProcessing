#ifndef SOFACV_IMGPROC_CROP_H
#define SOFACV_IMGPROC_CROP_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API Crop : public ImageFilter
{
 public:
  SOFA_CLASS(Crop, ImageFilter);

  sofa::Data<sofa::defaulttype::Vec4i> d_roi;

  Crop();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_CROP_H
