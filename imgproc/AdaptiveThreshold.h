#ifndef SOFACV_IMGPROC_ADAPTIVETHRESHOLD_H
#define SOFACV_IMGPROC_ADAPTIVETHRESHOLD_H

#include "ImageProcessingPlugin.h"
#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API AdaptiveThreshold : public ImageFilter
{
 public:
  SOFA_CLASS(AdaptiveThreshold, ImageFilter);

  sofa::Data<double> d_max;
  sofa::Data<sofa::helper::OptionsGroup> d_adaptiveMethod;
  sofa::Data<sofa::helper::OptionsGroup> d_thresholdType;
  sofa::Data<int> d_blockSize;
  sofa::Data<double> d_C;

  AdaptiveThreshold();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_ADAPTIVETHRESHOLD_H
