#ifndef SOFACV_IMGPROC_MORPHOLOGYEX_H
#define SOFACV_IMGPROC_MORPHOLOGYEX_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API MorphologyEx : public common::ImageFilter
{
 public:
  SOFA_CLASS(MorphologyEx, common::ImageFilter);

  sofa::Data<int> d_ksize;
  sofa::Data<sofa::helper::OptionsGroup> d_operator;
  sofa::Data<sofa::helper::OptionsGroup> d_element;

  MorphologyEx();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};


}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_MORPHOLOGYEX_H
