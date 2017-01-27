#ifndef SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
#define SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H

#include "ImageFilter.h"
#include "initplugin.h"

#include <SofaORCommon/CalibLoader.h>
#include <SofaORCommon/CameraCalib.h>
#include <SofaORCommon/StereoCalib.h>
#include <SofaORCommon/cvDMatch.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FeatureTriangulator : public ImageFilter
{
 public:
  SOFA_CLASS(FeatureTriangulator, ImageFilter);

 public:
  FeatureTriangulator();
  virtual ~FeatureTriangulator();

  void init();
  void update();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug);
  void reinit();
  void triangulate(const cv::Point2f& l, const cv::Point2f& r,
                   defaulttype::Vec3d& p);

  void getCalibFromContext()
  {
    common::CalibLoader* lastCalib =
        this->getContext()->get<common::CalibLoader>();
    if (lastCalib)
    {
      d_extrinsics.setParent(&lastCalib->d_stereoCalib,
                             "@" + lastCalib->getPathName() + ".stereo_calib");
      d_camLeft.setParent(&lastCalib->d_leftCalib,
                          "@" + lastCalib->getPathName() + ".left_calib");
      d_camRight.setParent(&lastCalib->d_rightCalib,
                           "@" + lastCalib->getPathName() + ".right_calib");
      msg_info(getClassName() + "::init()")
          << "Triangulator: Calibration data initialized from graph";
    }
  }

  // DATA
  Data<bool> d_rectify;
  // INPUTS
  Data<common::StereoCalib> d_extrinsics;
  Data<common::CameraCalib> d_camLeft;
  Data<common::CameraCalib> d_camRight;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR;
  Data<helper::SVector<helper::SVector<common::cvDMatch> > > d_matches;
  // OUTPUTS
  Data<sofa::helper::vector<defaulttype::Vec3d> > d_pointCloud;

 private:
  cv::Mat_<double> R;
  cv::Mat_<double> T;
  cv::Mat_<double> cmL;
  cv::Mat_<double> cmR;
  cv::Mat_<double> dvL;
  cv::Mat_<double> dvR;

  cv::Point3d rectifyPoint(double x, double y, cv::Mat_<double> distortion_vec);

  cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u, cv::Matx34d P,
                                                  cv::Point3d u1,
                                                  cv::Matx34d P1);
  cv::Mat_<double> linearLSTriangulation(cv::Point3d u, cv::Matx34d P,
                                         cv::Point3d u1, cv::Matx34d P1);
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
