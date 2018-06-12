#include "CvtColor.h"
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{

CvtColor::CvtColor()
    : d_code(initData(&d_code, 6, "code",
                      "color space conversion code default is BGR2GRAY")),
      d_dstCn(initData(&d_dstCn, 0, "dstCn",
                       "[OPTIONAL] number of channels in the destination "
                       "image; if the parameter is 0, the number of the "
                       "channels is derived automatically from src and code"))
{
}

void CvtColor::init() { ImageFilter::init(); }

void CvtColor::applyFilter(const cv::Mat &in,
                                                       cv::Mat &out, bool)
{
  if (in.empty()) return;
  try
  {
    cv::cvtColor(in, out, d_code.getValue(), d_dstCn.getValue());
  }
  catch (cv::Exception &e)
  {
    msg_error(getName() + "::applyFilter()")
        << "Exception thrown by cv::cvtColor()" << e.what();
    out = in;
  }
}

SOFA_DECL_CLASS(CvtColor)

int CvtColorClass = sofa::core::RegisterObject(
                        "Converts an image from one color space to another.")
                        .add<CvtColor>();

}  // namespace imgproc
}  // namespace sofacv


