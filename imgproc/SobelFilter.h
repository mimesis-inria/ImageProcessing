#ifndef SOFACV_IMGPROC_SOBELFILTER_H
#define SOFACV_IMGPROC_SOBELFILTER_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API SobelFilter : public ImageFilter
{
 public:
  SOFA_CLASS(SobelFilter, ImageFilter);

  sofa::Data<int> d_ddepth;
  sofa::Data<int> d_xorder;
  sofa::Data<int> d_yorder;
  sofa::Data<int> d_ksize;
  sofa::Data<double> d_scale;
  sofa::Data<double> d_delta;
  sofa::Data<sofa::helper::OptionsGroup> d_bordertype;

  SobelFilter();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_SOBELFILTER_H
