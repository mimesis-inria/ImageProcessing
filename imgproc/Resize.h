#ifndef SOFACV_IMGPROC_RESIZE_H
#define SOFACV_IMGPROC_RESIZE_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API Resize : public common::ImageFilter
{
 public:
  SOFA_CLASS(Resize, common::ImageFilter);

  sofa::Data<sofa::defaulttype::Vec2i> d_size;
  sofa::Data<double> d_fx;
  sofa::Data<double> d_fy;
  sofa::Data<sofa::helper::OptionsGroup> d_interp;

  Resize();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_RESIZE_H
