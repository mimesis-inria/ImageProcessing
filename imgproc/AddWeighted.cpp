#include "AddWeighted.h"

namespace sofacv
{
namespace imgproc
{


AddWeighted::AddWeighted()
    : d_img2(initData(&d_img2, "img2", "Image to add to img"))
{
}

void AddWeighted::init() { ImageFilter::init(); }

void AddWeighted::applyFilter(const cv::Mat &in,
                                                          cv::Mat &out, bool)
{
  if (in.empty() || d_img2.getValue().empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: cv::add requires a source and dest image";
    return;
  }

  cv::add(in, d_img2.getValue(), out);
}

SOFA_DECL_CLASS(AddWeighted)

int AddWeightedClass =
    sofa::core::RegisterObject("OpenCV's AddWeighted function")
        .add<AddWeighted>();


}  // namespace imgproc
}  // namespace sofacv

