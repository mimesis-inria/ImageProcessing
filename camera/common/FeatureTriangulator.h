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

namespace sofaor
{
namespace processor
{
namespace cam
{
class FeatureTriangulator : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<FeatureTriangulator, StereoSettings,
																							sofa::BaseLink::FLAG_STOREPATH |
																									sofa::BaseLink::FLAG_STRONGLINK>
			StereoCamSettings;
	typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;
	typedef sofa::defaulttype::Vec3d Vec3d;

 public:
  SOFA_CLASS(FeatureTriangulator, common::ImplicitDataEngine);

 public:
  FeatureTriangulator();
  virtual ~FeatureTriangulator();

  void init();
  void update();
	// DATA
	sofa::Data<bool> d_rectify;

	// INPUTS
	StereoCamSettings l_cam;
	sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL;
	sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR;
	sofa::Data<sofa::helper::vector<common::cvDMatch> > d_matches;

  // OUTPUTS
	sofa::Data<sofa::helper::vector<Vec3d> > d_pointCloud;

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

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
