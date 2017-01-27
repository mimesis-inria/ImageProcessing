#ifndef SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
#define SOFA_OR_PROCESSOR_IMAGERECTIFIER_H

#include "ImageFilter.h"
#include "SofaORCommon/CalibLoader.h"
#include "SofaORCommon/CameraCalib.h"
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

  Data<common::CameraCalib> d_calib;

  ImageRectifier()
      : d_calib(initData(&d_calib, common::CameraCalib(), "calib",
                         "camera calibration data for image undistortion"))
  {
  }

  void getCalibFromContext()
  {
    common::CalibLoader* lastCalib = this->getContext()->get<common::CalibLoader>();
    if (lastCalib)
    {
      d_calib.setParent(&lastCalib->d_leftCalib, "@" + lastCalib->getPathName() + ".left_calib");
      msg_info(getClassName() + "::init()")
          << "ImageRectifier Note: no input alib given to the "
             "filter. Linking to last CalibLoader's "
             "left_calib.";
    }
  }

  void init()
  {
      if (!d_calib.isSet()) getCalibFromContext();

      addInput(&d_calib);

      ImageFilter::init();
  }
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty() || d_calib.getValue().cameraMatrix.empty()) return;
    cv::Mat_<double> cam(d_calib.getValue().cameraMatrix.getNbLines(),
                         d_calib.getValue().cameraMatrix.getNbLines());
    common::matrix::sofaMat2cvMat(d_calib.getValue().cameraMatrix, cam);
    cv::undistort(in, out, cam, d_calib.getValue().distCoefs);
  }
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
    core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace collision

}  // namespace component

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_IMAGERECTIFIER_H
