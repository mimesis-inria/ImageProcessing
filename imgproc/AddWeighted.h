#ifndef SOFACV_IMGPROC_ADDWEIGHTED_H
#define SOFACV_IMGPROC_ADDWEIGHTED_H

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API AddWeighted : public common::ImageFilter
{
 public:
  SOFA_CLASS(AddWeighted, common::ImageFilter);

  sofa::Data<cvMat> d_img2;

  AddWeighted();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_ADDWEIGHTED_H
