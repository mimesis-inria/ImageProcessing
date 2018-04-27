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

#include "DescriptorMatcher.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <opencv2/features2d.hpp>

namespace sofacv
{
namespace features
{
SOFA_DECL_CLASS(DescriptorMatcher)

int DescriptorMatcherClass =
    sofa::core::RegisterObject("OpenCV component matching descriptors")
        .add<DescriptorMatcher>();

DescriptorMatcher::DescriptorMatcher()
    : d_matcherType(initData(&d_matcherType, "matcher",
                             "type of matcher to use (BRUTEFORCE or FLANN).")),
      d_matchingAlgo(initData(
          &d_matchingAlgo, "algo",
          "matching algorithm to use (STANDARD, KNN_MATCH, RADIUS_MATCH).")),
      d_k(initData(&d_k, 2, "k",
                   "k Count of best matches found per each query descriptor or "
                   "less if a query descriptor has less than k possible "
                   "matches in total. If K == -1, all descriptors will be "
                   "matched together")),
      d_maxDistance(initData(
          &d_maxDistance, .0f, "maxDistance",
          "maxDistance Threshold for the distance between matched descriptors. "
          "Distance means here metric distance (e.g. Hamming distance), not "
          "the distance between coordinates (which is measured in Pixels)!")),
      d_mask(initData(&d_mask, cvMat(), "mask",
                      "Mask specifying permissible matches between an input "
                      "query and train matrices of descriptors.")),
      d_queryDescriptors(initData(&d_queryDescriptors, "descriptors1",
                                  "Query set of descriptors", false)),
      d_trainDescriptors(initData(&d_trainDescriptors, "descriptors2",
                                  "Train set of descriptors", false)),
      d_in2(initData(&d_in2, "img2", "second image, for debug", false)),
      d_kptsL(initData(&d_kptsL, "keypoints1",
                       "left image's keypoints, for debug", false)),
      d_kptsR(initData(&d_kptsR, "keypoints2",
                       "right image's keypoints, for debug", false)),
      d_matches(initData(&d_matches, "matches", "output array of matches", true,
                         true))
{
  addAlias(&d_matches, "matches_out");
  sofa::helper::OptionsGroup* t = d_matcherType.beginEdit();
  t->setNames(MatcherType_COUNT, "FLANN", "BRUTEFORCE");
  t->setSelectedItem(0);
  d_matcherType.endEdit();

  t = d_matchingAlgo.beginEdit();
  t->setNames(MatchingAlgo_COUNT, "STANDARD", "KNN_MATCH", "RADIUS_MATCH");
  t->setSelectedItem(0);
  d_matchingAlgo.endEdit();

  m_matchers[FLANN] = new FlannMatcher(this);
  m_matchers[BRUTEFORCE] = new BFMatcher(this);
}

DescriptorMatcher::~DescriptorMatcher() {}
void DescriptorMatcher::init()
{
  std::cout << getName() << std::endl;
  std::cout << "Matcher type: " << d_matcherType.getValue().getSelectedItem()
            << std::endl;

  matcherTypeChanged();
  trackData(&d_matcherType);

  addInput(&d_queryDescriptors);
  addInput(&d_trainDescriptors);

  addInput(&d_in2, true);
  addInput(&d_kptsL, true);
  addInput(&d_kptsR, true);
  addInput(&d_mask, true);

  addOutput(&d_img_out);
  addOutput(&d_matches);
  ImageFilter::init();
}

void DescriptorMatcher::Update()
{
  if (m_dataTracker.isDirty(d_matcherType))
      matcherTypeChanged();

  msg_warning_when(!d_queryDescriptors.getValue().rows ||
                       !d_trainDescriptors.getValue().rows,
                   "DescriptorMatcher::update()")
      << "Error: Empty descriptor matrix!";
  ImageFilter::update();

  sofa::helper::SVector<sofa::helper::SVector<cvDMatch> >* vec =
      d_matches.beginWriteOnly();
  vec->clear();
  if (!m_matches.empty() && !m_matches[0].empty())
  {
    for (std::vector<cv::DMatch>& matchVec : m_matches)
    {
      vec->push_back(sofa::helper::SVector<cvDMatch>());
      for (cv::DMatch& match : matchVec)
        vec->back().push_back(cvDMatch(match));
    }
  }
  d_matches.endEdit();
  std::cout << "end" << getName() << std::endl;
}

void DescriptorMatcher::applyFilter(const cv::Mat& in, cv::Mat& out, bool)
{
  if (d_queryDescriptors.getValue().empty() ||
      d_trainDescriptors.getValue().empty())
    return;
  unsigned m = d_matcherType.getValue().getSelectedId();
  if (!m_matchers[m]->acceptsBinary() &&
      (d_queryDescriptors.getValue().type() == 0 ||
       d_trainDescriptors.getValue().type() == 0))
  {
    msg_error("DescriptorMatcher::update")
        << "Cannot match binary descriptors with these settings. either use "
           "BRUTEFORCE matcher or FLANN with LSHIndexParams";
    return;
  }
  else if (m_matchers[m]->acceptsBinary() &&
           (d_queryDescriptors.getValue().type() != 0 ||
            d_trainDescriptors.getValue().type() != 0))
  {
    msg_error("DescriptorMatcher::update")
        << "Cannot match non-binary descriptors with these settings.";
    return;
  }

  m_matches.clear();
  if (d_matchingAlgo.getValue().getSelectedId() == STANDARD_MATCH)
    m_matchers[m]->knnMatch(d_queryDescriptors.getValue(),
                            d_trainDescriptors.getValue(), m_matches, 1,
                            d_mask.getValue());
  else if (d_matchingAlgo.getValue().getSelectedId() == KNN_MATCH)
  {
    int k = d_k.getValue();
    int n = std::min(d_queryDescriptors.getValue().size[0],
                     d_trainDescriptors.getValue().size[0]);
    if (k > n || k == -1) k = n;

    m_matchers[m]->knnMatch(d_queryDescriptors.getValue(),
                            d_trainDescriptors.getValue(), m_matches, k,
                            d_mask.getValue());
  }
  else if (d_matchingAlgo.getValue().getSelectedId() == RADIUS_MATCH)
    m_matchers[m]->radiusMatch(d_queryDescriptors.getValue(),
                               d_trainDescriptors.getValue(), m_matches,
                               d_maxDistance.getValue(), d_mask.getValue());

  if (d_outputImage.getValue())
  {
    if (d_in2.isSet() && d_kptsL.isSet() && d_kptsR.isSet())
    {
      std::vector<cv::KeyPoint> kpL, kpR;
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_kptsL.getValue().data());
      kpL.assign(arr, arr + d_kptsL.getValue().size());
      arr = dynamic_cast<const cv::KeyPoint*>(d_kptsR.getValue().data());
      kpR.assign(arr, arr + d_kptsR.getValue().size());

      std::cout << m_matches.size() << std::endl;
      cv::drawMatches(in, kpL, d_in2.getValue(), kpR, m_matches, out);
    }
  }
}

void DescriptorMatcher::matcherTypeChanged()
{
  for (size_t i = 0; i < MatcherType_COUNT; ++i)
  {
    if (i == d_matcherType.getValue().getSelectedId())
    {
      m_matchers[i]->toggleVisible(true);
      m_matchers[i]->init();
    }
    else
      m_matchers[i]->toggleVisible(false);
  }
}

}  // namespace features
}  // namespace sofacv
