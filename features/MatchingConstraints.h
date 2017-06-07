#ifndef SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H
#define SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H

#include "Detectors.h"
#include "common/ImageFilter.h"
#include "camera/common/StereoSettings.h"

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
class MatchingConstraints : public ImageFilter
{
	typedef sofa::core::objectmodel::SingleLink<MatchingConstraints, StereoSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

public:
  SOFA_CLASS(MatchingConstraints, ImageFilter);

 public:
  MatchingConstraints();
  virtual ~MatchingConstraints();

  void init();
  void update();
	void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

  // INPUTS
	CamSettings l_cam;
  Data<bool> d_useEpipolarFilter;
  Data<int> d_epipolarThreshold;
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
	Data<helper::vector<common::cvDMatch> > d_matches_out;
  Data<sofa::helper::vector<size_t> > d_outliers_out;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL_out;
  Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR_out;
  Data<common::cvMat> d_descriptorsL_out;
  Data<common::cvMat> d_descriptorsR_out;

 private:
	helper::vector<common::cvDMatch> m_matches;
	helper::vector<common::cvKeypoint> m_kpL, m_kpR;
	common::cvMat m_descL, m_descR;
	helper::vector<size_t> m_outliers_out;

	std::vector<cv::Vec3f> m_epilines;
	float m_maxDist;

	struct MatchStruct
	{
		unsigned idxR;
		unsigned distance;
		unsigned distanceToEpiline;
	};

	struct MatchVector
	{
		unsigned idxL;
		std::vector<MatchStruct> matches;
	};
	std::vector<MatchVector> m_matchVector;

	bool computeEpipolarLines();

	bool EpipolarConstraintFilter(unsigned& filteredByEpipolar, unsigned i, MatchVector& ms, double epiDist);
	bool KNearestNeighborFilter(unsigned i, double lambda, MatchVector& ms, unsigned& filteredByKNN);
	bool MinimalDistanceFilter(MatchVector& ms, unsigned& filteredByMDF, unsigned i, float mdfDist);
	void PushInlier(const common::cvMat& descL, unsigned i, const helper::vector<common::cvKeypoint>& PointsL, const common::cvMat& descR, MatchVector& ms, const helper::vector<common::cvKeypoint>& PointsR);
	void ClearOutputVectors();
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H
