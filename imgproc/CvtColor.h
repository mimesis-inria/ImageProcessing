#ifndef SOFACV_IMGPROC_CVTCOLOR_H
#define SOFACV_IMGPROC_CVTCOLOR_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

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
class SOFA_IMAGEPROCESSING_API CvtColor : public common::ImageFilter
{
 public:
  SOFA_CLASS(CvtColor, common::ImageFilter);

  sofa::Data<int> d_code;
  sofa::Data<int> d_dstCn;

  CvtColor();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_CVTCOLOR_H
