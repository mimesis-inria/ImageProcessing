#ifndef SOFACV_IMGPROC_CANNYFILTER_H
#define SOFACV_IMGPROC_CANNYFILTER_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API CannyFilter : public common::ImageFilter
{
 public:
  SOFA_CLASS(CannyFilter, common::ImageFilter);

  sofa::Data<double> d_minThreshold;
  sofa::Data<double> d_maxThreshold;
  sofa::Data<int> d_apertureSize;
  sofa::Data<bool> d_l2gradient;

  CannyFilter();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_CANNYFILTER_H
