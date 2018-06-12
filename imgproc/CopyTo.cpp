#include "CopyTo.h"

namespace sofacv
{
namespace imgproc
{


CopyTo::CopyTo()
    : d_mask(initData(&d_mask, "mask", "mask")),
      d_useMask(initData(&d_useMask, true, "useMask",
                         "whether or not to use the input mask"))
{
}

void CopyTo::applyFilter(const cv::Mat &in,
                                                     cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: copyTo requires a source and dest image";
    return;
  }

  in.copyTo(out,
            (d_useMask.getValue()) ? (d_mask.getValue()) : (cvMat()));
}

void CopyTo::init()
{
  registerData(&d_useMask);
  ImageFilter::init();
}

SOFA_DECL_CLASS(CopyTo)

int CopyToClass =
    sofa::core::RegisterObject("OpenCV's CopyTo function").add<CopyTo>();



}  // namespace imgproc
}  // namespace sofacv

