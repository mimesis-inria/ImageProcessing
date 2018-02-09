/******************************************************************************
 *       SOFAOR, SOFA plugin for the Operating Room, development version       *
 *                        (c) 2017 INRIA, MIMESIS Team                         *
 *                                                                             *
 * This program is a free software; you can redistribute it and/or modify it   *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 1.0 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: Bruno Marques and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact-mimesis@inria.fr                               *
 ******************************************************************************/

#ifndef SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H
#define SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H

#include "Detectors.h"
#include "camera/common/StereoSettings.h"
#include "common/ImageFilter.h"

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
namespace features
{
class MatchingConstraints : public ImageFilter
{
  typedef sofa::core::objectmodel::SingleLink<
      MatchingConstraints, cam::StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
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
  sofa::Data<bool> d_useEpipolarFilter;
  sofa::Data<int> d_epipolarThreshold;
  sofa::Data<bool> d_useMDFilter;
  sofa::Data<float> d_mdfRadius;
  sofa::Data<bool> d_useKNNFilter;
  sofa::Data<float> d_knnLambda;
  sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL_in;
  sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR_in;
  sofa::Data<common::cvMat> d_descriptorsL_in;
  sofa::Data<common::cvMat> d_descriptorsR_in;
  sofa::Data<sofa::helper::SVector<sofa::helper::SVector<common::cvDMatch> > >
      d_matches_in;

  // OUTPUTS
  sofa::Data<sofa::helper::vector<common::cvDMatch> > d_matches_out;
  sofa::Data<sofa::helper::vector<size_t> > d_outliers_out;
  sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsL_out;
  sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypointsR_out;
  sofa::Data<common::cvMat> d_descriptorsL_out;
  sofa::Data<common::cvMat> d_descriptorsR_out;

 private:
  sofa::helper::vector<common::cvDMatch> m_matches;
  sofa::helper::vector<common::cvKeypoint> m_kpL, m_kpR;
  common::cvMat m_descL, m_descR;
  sofa::helper::vector<size_t> m_outliers_out;

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

  bool EpipolarConstraintFilter(unsigned& filteredByEpipolar, unsigned i,
                                MatchVector& ms, double epiDist);
  bool KNearestNeighborFilter(unsigned i, double lambda, MatchVector& ms,
                              unsigned& filteredByKNN);
  bool MinimalDistanceFilter(MatchVector& ms, unsigned& filteredByMDF,
                             unsigned i, float mdfDist);
  void PushInlier(const common::cvMat& descL, unsigned i,
                  const sofa::helper::vector<common::cvKeypoint>& PointsL,
                  const common::cvMat& descR, MatchVector& ms,
                  const sofa::helper::vector<common::cvKeypoint>& PointsR);
  void ClearOutputVectors();
};

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_MATCHINGCONSTRAINTS_H
