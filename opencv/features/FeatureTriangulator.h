#ifndef SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
#define SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvDMatch.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FeatureTriangulator : public common::ImplicitDataEngine
{
 public:
  SOFA_CLASS(FeatureTriangulator, common::ImplicitDataEngine);

 public:
  FeatureTriangulator();
  virtual ~FeatureTriangulator();

  void init();
  void update();
  void reinit();
  void triangulate(const cv::Point2f& l, const cv::Point2f& r,
                   defaulttype::Vec3d& p);

  // DATA
  Data<bool> d_rectify;
  // INPUTS
  Data<defaulttype::Matrix3> d_R;
  Data<defaulttype::Vector3> d_T;
  Data<defaulttype::Matrix3> d_cmL;
  Data<defaulttype::Matrix3> d_cmR;
  Data<helper::vector<double> > d_dvL;
  Data<helper::vector<double> > d_dvR;

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
  cv::Matx34d PL;
  cv::Matx34d PR;

  cv::Point3d rectifyPoint(double x, double y, cv::Mat_<double> distortion_vec);

  cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u,
                                                  cv::Point3d u1);
  cv::Mat_<double> linearLSTriangulation(cv::Point3d u, cv::Point3d u1);
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
