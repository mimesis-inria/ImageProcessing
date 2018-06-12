#ifndef SOFACV_IMGPROC_INRANGE_H
#define SOFACV_IMGPROC_INRANGE_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API InRange : public common::ImageFilter
{
  typedef typename sofa::defaulttype::Vec3i Vec3i;

 public:
  SOFA_CLASS(InRange, common::ImageFilter);

  sofa::Data<Vec3i> d_minRange;
  sofa::Data<Vec3i> d_maxRange;

  InRange();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_INRANGE_H
