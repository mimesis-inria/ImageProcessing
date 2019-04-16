#include "InRange.h"

namespace sofacv
{
namespace imgproc
{
InRange::InRange()
    : d_minRange(initData(&d_minRange, Vec3i(0, 0, 0), "minRange",
                          "minimum color value")),
      d_maxRange(initData(&d_maxRange, Vec3i(255, 255, 255), "maxRange",
                          "max color value"))
{
}

void InRange::init()
{
  registerData(&d_minRange, 0, 255, 1);
  registerData(&d_maxRange, 0, 255, 1);
  ImageFilter::init();
}

void InRange::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: cv::add requires a source and dest image";
    return;
  }

  cv::Scalar min(d_minRange.getValue().x(), d_minRange.getValue().y(),
                 d_minRange.getValue().z());
  cv::Scalar max(d_maxRange.getValue().x(), d_maxRange.getValue().y(),
                 d_maxRange.getValue().z());

  in.copyTo(out);
  cv::inRange(in, min, max, out);
}

SOFA_DECL_CLASS(InRange)

int InRangeClass =
    sofa::core::RegisterObject("OpenCV's InRange function").add<InRange>();

}  // namespace imgproc
}  // namespace sofacv
