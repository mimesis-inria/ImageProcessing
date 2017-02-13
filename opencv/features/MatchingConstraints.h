#ifndef SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H
#define SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H

#include "Detectors.h"
#include "core/ImageFilter.h"

#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvDMatch.h>
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
class MatchingConstraints : public ImageFilter
{
 public:
  SOFA_CLASS(MatchingConstraints, ImageFilter);

 public:
  MatchingConstraints();
  virtual ~MatchingConstraints();

  void init();
  void update();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug);
  void reinit();

  // INPUTS
  Data<bool> d_useEpipolarFilter;
  Data<int> d_epipolarThreshold;
  Data<defaulttype::Matrix3> d_F;
  Data<bool> d_useMDFilter;
  Data<float> d_mdfRadius;
  Data<bool> d_useKNNFilter;
  Data<float> d_knnLambda;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL_in;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR_in;
  Data<common::cvMat> d_descriptorsL_in;
  Data<common::cvMat> d_descriptorsR_in;
  Data<helper::SVector<helper::SVector<common::cvDMatch> > > d_matches_in;

  // OUTPUTS
  Data<helper::SVector<helper::SVector<common::cvDMatch> > > d_matches_out;
  Data<sofa::helper::vector<size_t> > d_outliers_out;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL_out;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR_out;
  Data<common::cvMat> d_descriptorsL_out;
  Data<common::cvMat> d_descriptorsR_out;

  // epipolar-specific outputs
  Data<sofa::helper::vector<defaulttype::Vec3f> > d_epilinesL;
  Data<sofa::helper::vector<defaulttype::Vec3f> > d_epilinesR;
  Data<sofa::helper::vector<float> > d_epidistL;
  Data<sofa::helper::vector<float> > d_epidistR;

  // mdf-specific outputs
  Data<sofa::helper::vector<float> > d_mdfDistances;
  Data<float> d_mdfMaxDist;

  // knn-specific outputs
  Data<sofa::helper::vector<float> > d_knnLambdas;

private:
  bool computeEpipolarLines();
  void computeEpipolarDistances();

  std::vector<std::vector<cv::DMatch> > m_matches_out;
  std::vector<cv::KeyPoint> m_kptsL, m_kptsR;
  cv::Mat m_descL, m_descR;

  std::vector<cv::Vec3f> m_epilinesL, m_epilinesR;
  std::vector<float> m_epidistanceL, m_epidistanceR;
  float m_maxDist;
  std::vector<float> m_mdfDistances;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H
