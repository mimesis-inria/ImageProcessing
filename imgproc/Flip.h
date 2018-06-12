#ifndef SOFACV_IMGPROC_FLIP_H
#define SOFACV_IMGPROC_FLIP_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{

class SOFA_IMAGEPROCESSING_API Flip : public common::ImageFilter
{
 public:
  SOFA_CLASS(Flip, common::ImageFilter);

  sofa::Data<int> d_flipCode;

  Flip();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_FLIP_H
