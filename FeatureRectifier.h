#ifndef SOFA_OR_PROCESSOR_FEATURERECTIFIER_H
#define SOFA_OR_PROCESSOR_FEATURERECTIFIER_H

#include "ImageFilter.h"
#include "initplugin.h"

#include <SofaORCommon/CalibLoader.h>
#include <SofaORCommon/CameraCalib.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FeatureRectifier : public ImageFilter
{
 public:
  SOFA_CLASS(FeatureRectifier, ImageFilter);

 public:
  FeatureRectifier();
  virtual ~FeatureRectifier();

  void init();
  void update();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug);
  void reinit();

  void getCalibFromContext()
  {
    common::CalibLoader* lastCalib =
        this->getContext()->get<common::CalibLoader>();
    if (lastCalib)
    {
      d_calib.setParent(&lastCalib->d_leftCalib,
                          "@" + lastCalib->getPathName() + ".left_calib");
      msg_info(getClassName() + "::init()")
          << "Rectifier: Calibration data initialized from graph";
    }
  }

  // INPUTS
  Data<common::CameraCalib> d_calib;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypoints_in;
  // OUTPUTS
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypoints_out;

 private:
  cv::Mat_<double> dist;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATURERECTIFIER_H
