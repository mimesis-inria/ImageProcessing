#ifndef SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
#define SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H

#include "StereoSettings.h"

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
	typedef sofa::core::objectmodel::SingleLink<FeatureTriangulator, StereoSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			StereoCamSettings;
	typedef defaulttype::Vec<3, uint8_t> Vec3b;
	typedef defaulttype::Vec3d Vec3d;

 public:
  SOFA_CLASS(FeatureTriangulator, common::ImplicitDataEngine);

 public:
  FeatureTriangulator();
  virtual ~FeatureTriangulator();

  void init();
  void update();
	// DATA
	Data<bool> d_rectify;

	// INPUTS
	StereoCamSettings l_cam;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR;
	Data<helper::vector<common::cvDMatch> > d_matches;

  // OUTPUTS
  Data<sofa::helper::vector<Vec3d> > d_pointCloud;

 private:
  cv::Mat_<double> R;
  cv::Mat_<double> T;
  cv::Mat_<double> cmL;
  cv::Mat_<double> cmR;
  cv::Mat_<double> dvL;
  cv::Mat_<double> dvR;
  cv::Matx34d PL;
  cv::Matx34d PR;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
