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

#ifndef SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H
#define SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H

#include "Matchers.h"
#include "common/ImageFilter.h"

#include <SofaORCommon/cvDMatch.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
class DescriptorMatcher : public ImageFilter
{
  enum MatcherType
  {
    FLANN = 0,
    BRUTEFORCE = 1,
    MatcherType_COUNT
  };

  enum MatchingAlgo
  {
    STANDARD_MATCH = 0,
    KNN_MATCH = 1,
    RADIUS_MATCH = 2,
    MatchingAlgo_COUNT
  };

 public:
  SOFA_CLASS(DescriptorMatcher, ImageFilter);

  DescriptorMatcher();
  virtual ~DescriptorMatcher();

  void init();
  void update();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug);

  sofa::Data<sofa::helper::OptionsGroup> d_matcherType;
  sofa::Data<sofa::helper::OptionsGroup> d_matchingAlgo;
  sofa::Data<int> d_k;
  sofa::Data<float> d_maxDistance;
  sofa::Data<common::cvMat> d_mask;

  sofa::Data<common::cvMat> d_queryDescriptors;
  sofa::Data<common::cvMat> d_trainDescriptors;

  sofa::Data<common::cvMat> d_in2;
  sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_kptsL;
  sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_kptsR;

  sofa::Data<sofa::helper::SVector<sofa::helper::SVector<common::cvDMatch> > >
      d_matches;

  void match(const common::cvMat& queryDescriptors,
             std::vector<cv::DMatch>& matches);
  void knnMatch(const common::cvMat& queryDescriptors,
                std::vector<std::vector<cv::DMatch> >& matches, int k);
  void radiusMatch(const common::cvMat& queryDescriptors,
                   std::vector<std::vector<cv::DMatch> >& matches,
                   float maxDistance);

 protected:
  void matcherTypeChanged(sofa::core::objectmodel::BaseData*);

 private:
  BaseMatcher* m_matchers[MatcherType_COUNT];
  std::vector<std::vector<cv::DMatch> > m_matches;
};

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H
