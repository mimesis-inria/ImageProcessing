#include "Resize.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{
Resize::Resize()
    : d_size(initData(&d_size, "size", "pixel resolution of the output image")),
      d_fx(initData(&d_fx, 0.0, "fx", "scale factor on the X axis")),
      d_fy(initData(&d_fy, 0.0, "fy", "scale factori on the Y axis")),
      d_interp(initData(&d_interp, "interpolation", "interpolation method"))
{
  sofa::helper::OptionsGroup *opt = d_interp.beginEdit();
  opt->setNames(5, "NEAREST", "LINEAR", "CUBIC", "AREA", "LANCZOS4");
  opt->setSelectedItem(1);
  d_interp.endEdit();
}

void Resize::init()
{
  addInput(&d_size);
  addInput(&d_fx);
  addInput(&d_fy);
  addInput(&d_interp);
  registerData(&d_interp);
  ImageFilter::init();
}

void Resize::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: Resize requires a source and dest image";
    return;
  }
  cv::Size s;

  if (d_size.isSet())
  {
    s.width = d_size.getValue().x();
    s.height = d_size.getValue().y();
  }
  cv::resize(in, out, s, d_fx.getValue(), d_fy.getValue(),
             int(d_interp.getValue().getSelectedId()));
}

SOFA_DECL_CLASS(Resize)

int ResizeClass =
    sofa::core::RegisterObject("OpenCV's Resize function").add<Resize>();

}  // namespace imgproc
}  // namespace sofacv
