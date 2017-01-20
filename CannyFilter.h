#include <opencv2/imgproc.hpp>
#include "ImageFilter.h"

namespace sofa
{
namespace OR
{
namespace processor
{
class CannyFilter : public ImageFilter
{
 public:
  SOFA_CLASS(CannyFilter, ImageFilter);

  Data<double> d_minThreshold;
  Data<double> d_maxThreshold;
  Data<int> d_apertureSize;
  Data<bool> d_l2gradient;

  CannyFilter()
      : d_minThreshold(
            initData(&d_minThreshold, 0.0, "min",
                     "first threshold for the hysteresis procedure.")),
        d_maxThreshold(
            initData(&d_maxThreshold, 80.0, "max",
                     "second threshold for the hysteresis procedure.")),
        d_apertureSize(initData(&d_apertureSize, 3, "apertureSize",
                                "Canny's aperture size")),
        d_l2gradient(initData(&d_l2gradient, false, "L2gradient",
                              "more precision when set to true"))
  {
  }

  void init()
  {
    registerData(&d_minThreshold, 0.0, 255.0, 1);
    registerData(&d_maxThreshold, 0.0, 255.0, 1);
    registerData(&d_apertureSize, 0, 10, 1);
    registerData(&d_l2gradient);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out)
  {
    if (in.empty()) return;
    cv::Mat img_grey;
    cv::cvtColor(in, img_grey, CV_BGRA2GRAY);

    cv::Canny(img_grey, img_grey, d_minThreshold.getValue(),
              d_maxThreshold.getValue(), d_apertureSize.getValue(),
              d_l2gradient.getValue());

    cv::cvtColor(img_grey, out, CV_GRAY2BGRA);
  }
};

}  // namespace collision

}  // namespace component

}  // namespace sofa
