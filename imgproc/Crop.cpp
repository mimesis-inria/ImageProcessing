#include "Crop.h"

namespace sofacv
{
namespace imgproc
{

Crop::Crop()
    : d_roi(initData(&d_roi, "ROI", "x, y, w, h values of the ROI"))
{
}

void Crop::applyFilter(const cv::Mat &in,
                                                   cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: Resize requires a source and dest image";
    return;
  }
  cv::Rect roi;
  if (!d_roi.isSet())
    roi = cv::Rect(0, 0, in.cols, in.rows);
  else
    roi = cv::Rect(d_roi.getValue()[0], d_roi.getValue()[1],
                   d_roi.getValue()[2], d_roi.getValue()[3]);

  out = in(roi).clone();
}

void Crop::init()
{
  registerData(&d_roi, 0, 2000, 1);
  ImageFilter::init();
}

SOFA_DECL_CLASS(Crop)

int CropClass =
    sofa::core::RegisterObject("OpenCV's Crop function").add<Crop>();

}  // namespace imgproc
}  // namespace sofacv

