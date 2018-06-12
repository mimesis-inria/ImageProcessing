#include "CannyFilter.h"
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{


CannyFilter::CannyFilter()
    : d_minThreshold(initData(&d_minThreshold, 0.0, "min",
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

void CannyFilter::init()
{
  addInput(&d_minThreshold, true);
  registerData(&d_minThreshold, 0.0, 255.0, 1.0);
  addInput(&d_maxThreshold, true);
  registerData(&d_maxThreshold, 0.0, 255.0, 1.0);
  addInput(&d_apertureSize, true);
  registerData(&d_apertureSize, 3, 7, 2);
  addInput(&d_l2gradient, true);
  registerData(&d_l2gradient);
  ImageFilter::init();
}

void CannyFilter::applyFilter(const cv::Mat &in,
                                                          cv::Mat &out, bool)
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

  cv::Mat img_gray;
  if (in.type() == CV_8UC4)
    cv::cvtColor(in, img_gray, CV_BGRA2GRAY);
  else if (in.type() == CV_8UC3)
    cv::cvtColor(in, img_gray, CV_BGR2GRAY);
  else
    img_gray = in.clone();

  cv::Canny(img_gray, img_gray, d_minThreshold.getValue(),
            d_maxThreshold.getValue(), d_apertureSize.getValue(),
            d_l2gradient.getValue());

  out = img_gray;
}

SOFA_DECL_CLASS(CannyFilter)

int CannyFilterClass =
    sofa::core::RegisterObject("Canny edge detection filter from OpenCV")
        .add<CannyFilter>();

}  // namespace imgproc
}  // namespace sofacv


