#include "Fill.h"

namespace sofacv
{
namespace imgproc
{
Fill::Fill()
    : d_color(initData(&d_color, sofa::defaulttype::Vec4d(1.0, 1.0, 1.0, 1.0),
                       "scalar", "pixel color value."))
{
}

void Fill::init()
{
  registerData(&d_color, 0.0, 1.0, 0.0001);
  ImageFilter::init();
}

void Fill::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
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

SOFA_DECL_CLASS(Fill)

int FillClass =
    sofa::core::RegisterObject("OpenCV's implementation of cv::Mat::setTo()")
        .add<Fill>();

}  // namespace imgproc
}  // namespace sofacv
