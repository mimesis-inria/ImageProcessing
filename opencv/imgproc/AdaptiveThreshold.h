#ifndef SOFA_OR_PROCESSOR_ADAPTIVETHRESHOLD_H
#define SOFA_OR_PROCESSOR_ADAPTIVETHRESHOLD_H

#include <opencv2/imgproc.hpp>
#include "core/ImageFilter.h"

namespace sofa
{
namespace OR
{
namespace processor
{
class AdaptiveThreshold : public ImageFilter
{
 public:
  SOFA_CLASS(AdaptiveThreshold, ImageFilter);

  Data<helper::OptionsGroup> d_adaptiveMethod;
  Data<helper::OptionsGroup> d_thresholdType;
  Data<int> d_blockSize;
  Data<double> d_max;
  Data<double> d_C;

  AdaptiveThreshold()
      : d_max(initData(&d_max, 255.0, "maxValue",
                       "non-zero value assigned to the pixels for which the "
                       "condition is satisfied")),
        d_adaptiveMethod(initData(
            &d_adaptiveMethod, "adaptiveMethod",
            "Adaptive thresholding algorithm to use (MEAN or GAUSSIAN)")),
        d_thresholdType(initData(&d_thresholdType, "thresholdType",
                                 "either BINARY or BINARY_INV")),
        d_blockSize(initData(&d_blockSize, 7, "blockSize",
                             "size of the neighborhood that is used to "
                             "calculate the threshold value (3,5,7 and so "
                             "on...)")),
        d_C(initData(&d_C, 5.0, "C",
                     "Constant substracted from the mean or weighted mean -see "
                     "the details below).normally it is possitive but may be "
                     "zero or negative as well."))
  {
    helper::OptionsGroup* t = d_adaptiveMethod.beginEdit();
    t->setNames(2, "MEAN", "GAUSSIAN");
    t->setSelectedItem("GAUSSIAN");
    d_adaptiveMethod.endEdit();
    t = d_thresholdType.beginEdit();
    t->setNames(2, "BINARY", "BINARY_INV");
    t->setSelectedItem("BINARY");
    d_thresholdType.endEdit();
  }

  void init()
  {
    registerData(&d_adaptiveMethod, 0, 1, 1);
    registerData(&d_thresholdType, 0, 1, 1);
    registerData(&d_max, 0.0, 1.0, .0001);
    registerData(&d_blockSize, 3, 51, 2);
    registerData(&d_C, -10.0, 10.0, 1.0);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;

    cv::Mat img;
    if (in.type() == CV_32FC1)
    {
        msg_warning("ImageExporter::export()")
            << "CV_32F matrices will be normalized into a CV_8U matrix. Consider "
               "converting first to optimize performances";
      cv::normalize(in, img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }
    else if (in.type() != CV_8UC1)
    {
      msg_error("AdaptiveThreshold::applyFliter()")
          << "Error: Threshold can only be applied on 8-bit single "
             "channel images";
      return;
    }
    else
        img = in;
    try
    {
      cv::adaptiveThreshold(img, out, d_max.getValue() * 255,
                            d_adaptiveMethod.getValue().getSelectedId(),
                            d_thresholdType.getValue().getSelectedId(),
                            d_blockSize.getValue(), d_C.getValue());
    }
    catch (cv::Exception& e)
    {
      std::cout << e.what() << std::endl;
      return;
    }
  }
};

SOFA_DECL_CLASS(AdaptiveThreshold)

int AdaptiveThresholdClass =
    core::RegisterObject(
        "OpenCV's implementation of an adaptive image thresholding filter")
        .add<AdaptiveThreshold>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_ADAPTIVETHRESHOLD_H
