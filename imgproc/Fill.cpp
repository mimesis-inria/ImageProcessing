#include "Fill.h"

sofaor::processor::imgproc::Fill::Fill()
    : d_color(initData(&d_color, sofa::defaulttype::Vec4d(1.0, 1.0, 1.0, 1.0),
                       "scalar", "pixel color value."))
{
}

void sofaor::processor::imgproc::Fill::init()
{
  registerData(&d_color, 0.0, 1.0, 0.0001);
  ImageFilter::init();
}

void sofaor::processor::imgproc::Fill::applyFilter(const cv::Mat &in,
                                                   cv::Mat &out, bool)
{
  if (in.empty()) return;

  sofa::defaulttype::Vec4d color;
  if (in.depth() == CV_8U)
  {
    color = d_color.getValue() * 255;
  }
  else if (in.depth() == CV_32F)
  {
    color = d_color.getValue();
  }
  try
  {
    in.copyTo(out);
    out.setTo(cv::Scalar(color.x(), color.y(), color.z(), color.w()));
  }
  catch (cv::Exception &e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
}
