#ifndef SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
#define SOFA_OR_PROCESSOR_IMAGERECTIFIER_H

#include "core/ImageFilter.h"
#include "SofaORCommon/cvMatUtils.h"

#include <opencv2/imgproc.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class ImageRectifier : public ImageFilter
{
 public:
  SOFA_CLASS(ImageRectifier, ImageFilter);

  Data<defaulttype::Matrix3> d_projMat;
  Data<helper::vector<double> > d_distCoefs;

  ImageRectifier()
      : d_projMat(initData(&d_projMat, "projMat",
                           "3x3 projection matrix (from camera's "
                           "intrinsic parameters)")),
        d_distCoefs(initData(&d_distCoefs, "distCoefs",
                             "distortion coefficients vector (from camera's "
                             "intrinsic parameters)"))
  {
  }

  void init()
  {
    std::cout << getName()<< "init" << std::endl;

    addInput(&d_distCoefs);
    ImageFilter::init();
  }
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    std::cout << getName() << "applyFilter " << std::endl;;

    if (in.empty() || d_distCoefs.getValue().empty()) return;
    cv::Mat_<double> cam;
    common::matrix::sofaMat2cvMat(d_projMat.getValue(), cam);
    cv::undistort(in, out, cam, d_distCoefs.getValue());
  }
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
    core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
