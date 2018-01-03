#include "MinMaxLoc.h"

sofaor::processor::imgproc::MinMaxLoc::MinMaxLoc()
    : d_min(initData(&d_min, "min", "min score", true, true)),
      d_max(initData(&d_max, "max", "max score", true, true)),
      d_minLoc(initData(&d_minLoc, "minLoc",
                        "pointer to the min score's location", true, true)),
      d_maxLoc(initData(&d_maxLoc, "maxLoc",
                        "pointer to the max score's location", true, true)),
      d_mask(initData(&d_mask, "mask", "image mask to reduce searching area"))
{
}

void sofaor::processor::imgproc::MinMaxLoc::init() { ImageFilter::init(); }

void sofaor::processor::imgproc::MinMaxLoc::applyFilter(const cv::Mat &in,
                                                        cv::Mat &, bool)
{
  if (in.empty()) return;

  if (in.channels() != 1)
  {
    msg_error("MinMaxLoc::applyFilter()")
        << "MinMaxLoc takes single channelled images";
  }
  try
  {
    double min, max;
    cv::Point minl, maxl;
    cv::minMaxLoc(in, &min, &max, &minl, &maxl,
                  (d_mask.isSet()) ? (d_mask.getValue()) : (cv::noArray()));
    d_min.setValue(min);
    d_max.setValue(max);
    d_minLoc.setValue(sofa::defaulttype::Vec2i(minl.x, minl.y));
    d_maxLoc.setValue(sofa::defaulttype::Vec2i(maxl.x, maxl.y));
  }
  catch (cv::Exception &e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
}
