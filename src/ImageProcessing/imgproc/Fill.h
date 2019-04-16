#ifndef SOFACV_IMGPROC_FILL_H
#define SOFACV_IMGPROC_FILL_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API Fill : public ImageFilter
{
 public:
  SOFA_CLASS(Fill, ImageFilter);

  sofa::Data<sofa::defaulttype::Vec4d> d_color;

  Fill();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_FILL_H
