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
    registerData(&d_minThreshold, 0.0, 255.0, 1.0);
    registerData(&d_maxThreshold, 0.0, 255.0, 1.0);
    registerData(&d_apertureSize, 3, 7, 1);
    registerData(&d_l2gradient);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    int apertureSize = d_apertureSize.getValue();
    if (apertureSize != 3 && apertureSize != 5 && apertureSize != 7)
    {
      msg_warning("CannyFilter::applyFliter()")
          << "Error: Aperture Size should be either 3, 5 or 7.";
      return;
    }
    if (d_minThreshold.getValue() < 0.0 || d_minThreshold.getValue() > 255.0 ||
        d_maxThreshold.getValue() < 0.0 || d_maxThreshold.getValue() > 255.0)
    {
      msg_warning("CannyFilter::applyFliter()")
          << "Error: Thresholds should be between 0 - 255 as we're using 8-bit "
             "grayscale images.";
      return;
    }

    cv::Mat img_grey;
    cv::cvtColor(in, img_grey, CV_BGRA2GRAY);

    cv::Canny(img_grey, img_grey, d_minThreshold.getValue(),
              d_maxThreshold.getValue(), d_apertureSize.getValue(),
              d_l2gradient.getValue());

    cv::cvtColor(img_grey, out, CV_GRAY2BGRA);
  }
};

SOFA_DECL_CLASS(CannyFilter)

int CannyFilterClass =
    core::RegisterObject("Canny edge detection filter from OpenCV")
        .add<CannyFilter>();

}  // namespace collision

}  // namespace component

}  // namespace sofa
