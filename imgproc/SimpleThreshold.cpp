#include "SimpleThreshold.h"
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{

SimpleThreshold::SimpleThreshold()
    : d_threshold(initData(&d_threshold, .5, "threshold", "threshold value")),
      d_max(initData(&d_max, 1.0, "max",
                     "maximum value to use with the THRESH_BINARY and "
                     "THRESH_BINARY_INV thresholding types")),
      d_type(initData(&d_type, "thresholdType",
                      "thresholding type (see cv::ThresholdTypes"))
{
  sofa::helper::OptionsGroup* t = d_type.beginEdit();
  t->setNames(5, "THRESH_BINARY", "THRESH_BINARY_INV", "THRESH_TRUNC",
              "THRESH_TOZERO", "THRESH_TOZERO_INV");
  t->setSelectedItem("THRESH_BINARY");
  d_type.endEdit();
}

void SimpleThreshold::init()
{
  registerData(&d_threshold, 0.0, 1.0, .0001);
  registerData(&d_max, 0.0, 1.0, .0001);
  registerData(&d_type);
  ImageFilter::init();
}

void SimpleThreshold::applyFilter(const cv::Mat& in,
                                                              cv::Mat& out,
                                                              bool)
{
  if (in.empty()) return;

  double max, thresh;
  if (in.type() == CV_8UC1)
  {
    max = d_max.getValue() * 255;
    thresh = d_threshold.getValue() * 255;
  }
  else if (in.type() == CV_32FC1)
  {
    max = d_max.getValue();
    thresh = d_threshold.getValue();
  }
  else
  {
    msg_error("SimpleThreshold::applyFliter()")
        << "Error: Threshold can only be applied on 8-bit or 32-bit single "
           "channel images";
    return;
  }
  try
  {
    cv::threshold(in, out, thresh, max, d_type.getValue().getSelectedId());
  }
  catch (cv::Exception& e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
}

SOFA_DECL_CLASS(SimpleThreshold)

int SimpleThresholdClass =
    sofa::core::RegisterObject(
        "OpenCV's implementation of a simple image thresholding filter")
        .add<SimpleThreshold>();


}  // namespace imgproc
}  // namespace sofacv


