#ifndef SOFACV_IMGPROC_CVTCOLOR_H
#define SOFACV_IMGPROC_CVTCOLOR_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
/**
 * @brief Converts an image from one color space to another.
 *
 * Please refer to OpenCV's ColorConversionCodes enumeration in imgproc.hpp for
 * color codes
 */
class SOFA_IMAGEPROCESSING_API CvtColor : public ImageFilter
{
 public:
  SOFA_CLASS(CvtColor, ImageFilter);

  sofa::Data<int> d_code;
  sofa::Data<int> d_dstCn;

  CvtColor();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_CVTCOLOR_H
