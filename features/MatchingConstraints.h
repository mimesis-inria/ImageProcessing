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

#ifndef SOFACV_FEATURES_MATCHINGCONSTRAINTS_H
#define SOFACV_FEATURES_MATCHINGCONSTRAINTS_H

#include "Detectors.h"
#include "camera/common/StereoSettings.h"
#include "common/ImageFilter.h"

#include <SofaCV/SofaCV.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace features
{
class SOFA_IMAGEPROCESSING_API MatchingConstraints : public common::ImageFilter
{
  typedef sofa::core::objectmodel::SingleLink<
      MatchingConstraints, cam::StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(MatchingConstraints, ImageFilter);

 public:
  MatchingConstraints();
  virtual ~MatchingConstraints() override;

  void init() override;
  virtual void Update() override;
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;

  // INPUTS
  CamSettings l_cam;
  sofa::Data<bool> d_useEpipolarFilter;
  sofa::Data<int> d_epipolarThreshold;
  sofa::Data<bool> d_useMDFilter;
  sofa::Data<float> d_mdfRadius;
  sofa::Data<bool> d_useKNNFilter;
  sofa::Data<float> d_knnLambda;
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_keypointsL_in;
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_keypointsR_in;
  sofa::Data<cvMat> d_descriptorsL_in;
  sofa::Data<cvMat> d_descriptorsR_in;
  sofa::Data<sofa::helper::SVector<sofa::helper::SVector<cvDMatch> > >
      d_matches_in;

  // OUTPUTS
  sofa::Data<sofa::helper::vector<cvDMatch> > d_matches_out;
  sofa::Data<sofa::helper::vector<size_t> > d_outliers_out;
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_keypointsL_out;
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_keypointsR_out;
  sofa::Data<cvMat> d_descriptorsL_out;
  sofa::Data<cvMat> d_descriptorsR_out;

 private:
  sofa::helper::vector<cvDMatch> m_matches;
  sofa::helper::vector<cvKeypoint> m_kpL, m_kpR;
  cvMat m_descL, m_descR;
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
  void PushInlier(const cvMat& descL, unsigned i,
                  const sofa::helper::vector<cvKeypoint>& PointsL,
                  const cvMat& descR, MatchVector& ms,
                  const sofa::helper::vector<cvKeypoint>& PointsR);
  void ClearOutputVectors();
};

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_MATCHINGCONSTRAINTS_H
