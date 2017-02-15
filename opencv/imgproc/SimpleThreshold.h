#ifndef SOFA_OR_PROCESSOR_SIMPLETHRESHOLD_H
#define SOFA_OR_PROCESSOR_SIMPLETHRESHOLD_H

#include <opencv2/imgproc.hpp>
#include "core/ImageFilter.h"

namespace sofa
{
namespace OR
{
namespace processor
{
class SimpleThreshold : public ImageFilter
{
 public:
  SOFA_CLASS(SimpleThreshold, ImageFilter);

  Data<double> d_threshold;
  Data<double> d_max;
  Data<helper::OptionsGroup> d_type;

  SimpleThreshold()
      : d_threshold(
            initData(&d_threshold, 128.0, "threshold", "threshold value")),
        d_max(initData(&d_max, 255.0, "max",
                       "maximum value to use with the THRESH_BINARY and "
                       "THRESH_BINARY_INV thresholding types")),
        d_type(initData(&d_type, "type",
                        "thresholding type (see cv::ThresholdTypes"))
  {
    helper::OptionsGroup* t = d_type.beginEdit();
    t->setNames(5, "THRESH_BINARY", "THRESH_BINARY_INV", "THRESH_TRUNC",
                "THRESH_TOZERO", "THRESH_TOZERO_INV");
    t->setSelectedItem("THRESH_BINARY");
    d_type.endEdit();
  }

  void init()
  {
    registerData(&d_threshold, 0.0, d_max.getValue(), d_max.getValue() / 255.0);
    registerData(&d_max, 0.0, 255.0, 1.0);
    registerData(&d_type, 0, 5, 1);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    if (in.depth() != CV_8U || in.channels() != 1)
    {
      msg_error("SimpleThreshold::applyFliter()")
          << "Error: Threshold can only be applied on 8-bit grayscale images";
      return;
    }
    try {
      cv::threshold(in, out, d_threshold.getValue(), d_max.getValue(),
                    d_type.getValue().getSelectedId());
    } catch (cv::Exception& e) {
      std::cout << e.what() << std::endl;
      return;
    }
  }
};

SOFA_DECL_CLASS(SimpleThreshold)

int SimpleThresholdClass =
    core::RegisterObject("OpenCV's implementation of a simple image thresholding filter")
        .add<SimpleThreshold>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_SIMPLETHRESHOLD_H
