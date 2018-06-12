#ifndef SOFACV_IMGPROC_SIMPLETHRESHOLD_H
#define SOFACV_IMGPROC_SIMPLETHRESHOLD_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API SimpleThreshold : public common::ImageFilter
{
 public:
  SOFA_CLASS(SimpleThreshold, common::ImageFilter);

  sofa::Data<double> d_threshold;
  sofa::Data<double> d_max;
  sofa::Data<sofa::helper::OptionsGroup> d_type;

  SimpleThreshold();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_SIMPLETHRESHOLD_H
