#ifndef SOFACV_IMGPROC_COPYTO_H
#define SOFACV_IMGPROC_COPYTO_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API CopyTo : public common::ImageFilter
{
 public:
  SOFA_CLASS(CopyTo, common::ImageFilter);

  sofa::Data<cvMat> d_mask;
  sofa::Data<bool> d_useMask;

  CopyTo();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_COPYTO_H
