#ifndef SOFACV_IMGPROC_ADDWEIGHTED_H
#define SOFACV_IMGPROC_ADDWEIGHTED_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API AddWeighted : public ImageFilter
{
 public:
  SOFA_CLASS(AddWeighted, ImageFilter);

  sofa::Data<cvMat> d_img2;

  AddWeighted();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_ADDWEIGHTED_H
