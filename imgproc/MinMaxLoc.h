#ifndef SOFACV_IMGPROC_MINMAXLOC_H
#define SOFACV_IMGPROC_MINMAXLOC_H

#include "ImageProcessingPlugin.h"
#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API MinMaxLoc : public ImageFilter
{

 public:
  SOFA_CLASS(MinMaxLoc, ImageFilter);

  sofa::Data<double> d_min;
  sofa::Data<double> d_max;
  sofa::Data<sofa::defaulttype::Vec2i> d_minLoc;
  sofa::Data<sofa::defaulttype::Vec2i> d_maxLoc;
  sofa::Data<cvMat> d_mask;

  MinMaxLoc();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& /*out*/, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_MINMAXLOC_H
