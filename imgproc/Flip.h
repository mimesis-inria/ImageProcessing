#ifndef SOFACV_IMGPROC_FLIP_H
#define SOFACV_IMGPROC_FLIP_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API Flip : public ImageFilter
{
 public:
  SOFA_CLASS(Flip, ImageFilter);

  sofa::Data<int> d_flipCode;

  Flip();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_FLIP_H
