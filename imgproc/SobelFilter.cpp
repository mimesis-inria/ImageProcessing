#include "SobelFilter.h"
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{


SobelFilter::SobelFilter()
    : d_ddepth(initData(&d_ddepth, -1, "ddepth",
                        "Output image depth. Same as source by default (-1). "
                        "possible values: 1, 2, 4, 8 bytes")),
      d_xorder(initData(&d_xorder, 1, "dx", "order of the derivative x")),
      d_yorder(initData(&d_yorder, 1, "dy", "order of the derivative y")),
      d_ksize(initData(&d_ksize, 3, "ksize",
                       "Sobel's kernel size. Must be either 1,3,5 or 7. "
                       "Default value: 3")),
      d_scale(initData(&d_scale, 1.0, "scale",
                       "scale factor for the computed derivative values. By "
                       "default, no scale is applied")),
      d_delta(initData(&d_delta, 0.0, "delta",
                       "optional delta value that is added to the "
                       "restults prior to storing them in the output "
                       "image")),
      d_bordertype(
          initData(&d_bordertype, "borderType", "pixel interpolation method"))
{
  sofa::helper::OptionsGroup* t = d_bordertype.beginEdit();
  t->setNames(
      6, "CONSTANT",  //!< `iiiiii|abcdefgh|iiiiiii`  with some specified `i`
      "BORDER_REPLICATE",     //!< `aaaaaa|abcdefgh|hhhhhhh`
      "BORDER_REFLECT",       //!< `fedcba|abcdefgh|hgfedcb`
      "BORDER_WRAP",          //!< `cdefgh|abcdefgh|abcdefg`
      "BORDER_REFLECT_101",   //!< `gfedcb|abcdefgh|gfedcba`
      "BORDER_TRANSPARENT");  //!< `uvwxyz|absdefgh|ijklmno`
  t->setSelectedItem("BORDER_REFLECT_101");
  d_bordertype.endEdit();
}

void SobelFilter::init()
{
  registerData(&d_xorder, 0, 7, 1);
  registerData(&d_yorder, 0, 7, 1);
  registerData(&d_ksize, 1, 7, 2);
  registerData(&d_scale, 0.0, 100, 0.01);
  registerData(&d_delta, 0.0, 100.0, 0.01);
  registerData(&d_bordertype);
  ImageFilter::init();
}

void SobelFilter::applyFilter(const cv::Mat& in,
                                                          cv::Mat& out, bool)
{
  if (in.empty()) return;
  int ksize = d_ksize.getValue();
  if (ksize != 1 && ksize != 3 && ksize != 5 && ksize != 7)
  {
    msg_error("SobelFilter::applyFliter()")
        << "Error: Sobel's kernel size should be either 1, 3, 5 or 7.";
    return;
  }
  if (d_ddepth.getValue() != -1)
    if (d_ddepth.getValue() < in.depth())
    {
      msg_error("SobelFilter::applyFliter()")
          << "Error: output image's depth (ddepth) must be same or bigger "
             "than input image's depth. Set to -1 if you don't know";
      return;
    }
  if (d_ksize.getValue() < d_xorder.getValue() ||
      d_ksize.getValue() < d_yorder.getValue())
  {
    msg_error("SobelFilter::applyFliter()") << "Error: ksize must always be "
                                               "greater than x and y "
                                               "derivative orders";
    return;
  }

  try
  {
    cv::Sobel(in, out, d_ddepth.getValue(), d_xorder.getValue(),
              d_yorder.getValue(), d_ksize.getValue(), d_scale.getValue(),
              d_delta.getValue(), d_bordertype.getValue().getSelectedId());
  }
  catch (cv::Exception& e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
}

SOFA_DECL_CLASS(SobelFilter)

int SobelFilterClass =
    sofa::core::RegisterObject("OpenCV's implementation of the Sobel operator")
        .add<SobelFilter>();

}  // namespace imgproc
}  // namespace sofacv

