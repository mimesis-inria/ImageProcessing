#ifndef SOFACV_IMGPROC_INRANGE_H
#define SOFACV_IMGPROC_INRANGE_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API InRange : public ImageFilter
{
  typedef typename sofa::defaulttype::Vec3i Vec3i;

 public:
  SOFA_CLASS(InRange, ImageFilter);

  sofa::Data<Vec3i> d_minRange;
  sofa::Data<Vec3i> d_maxRange;

  InRange();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_INRANGE_H
