#ifndef SOFACV_IMGPROC_TEMPLATEMATCHER_H
#define SOFACV_IMGPROC_TEMPLATEMATCHER_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

#include <sofa/helper/OptionsGroup.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{
class SOFA_IMAGEPROCESSING_API TemplateMatcher : public ImageFilter
{
 public:
  SOFA_CLASS(TemplateMatcher, ImageFilter);

  sofa::Data<cvMat> d_template;
  sofa::Data<sofa::helper::OptionsGroup> d_method;

  TemplateMatcher();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_TEMPLATEMATCHER_H
