#include "Flip.h"

namespace sofacv
{
namespace imgproc
{
Flip::Flip()
    : d_flipCode(initData(&d_flipCode, 1, "flipCode",
                          "0 for X axis flip, 1 for Y, and -1 for both"))
{
}

void Flip::init()
{
  addInput(&d_flipCode, true);
  registerData(&d_flipCode, -1, 1, 1);
  ImageFilter::init();
}

void Flip::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: Flip requires a source and dest image";
    return;
  }

  cv::flip(in, out, d_flipCode.getValue());
}

SOFA_DECL_CLASS(Flip)

int FlipClass =
    sofa::core::RegisterObject("OpenCV's Flip function").add<Flip>();

}  // namespace imgproc
}  // namespace sofacv
