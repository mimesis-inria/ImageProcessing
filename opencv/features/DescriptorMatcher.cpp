#include "DescriptorMatcher.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <opencv2/features2d.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(DescriptorMatcher)

int DescriptorMatcherClass =
    core::RegisterObject("OpenCV component matching descriptors")
        .add<DescriptorMatcher>();

DescriptorMatcher::DescriptorMatcher()
    : ImageFilter(false),
      d_matcherType(initData(&d_matcherType, "matcher",
                             "type of matcher to use (BRUTEFORCE or FLANN).")),
      d_matchingAlgo(initData(
          &d_matchingAlgo, "algo",
          "matching algorithm to use (STANDARD, KNN_MATCH, RADIUS_MATCH).")),
      d_k(initData(&d_k, 2, "k",
                   "k Count of best matches found per each query descriptor or "
                   "less if a query descriptor has less than k possible "
                   "matches in total.")),
      d_maxDistance(initData(
          &d_maxDistance, .0f, "maxDistance",
          "maxDistance Threshold for the distance between matched descriptors. "
          "Distance means here metric distance (e.g. Hamming distance), not "
          "the distance between coordinates (which is measured in Pixels)!")),
      d_mask(initData(&d_mask, common::cvMat(), "mask",
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
  m_outputImage = false;
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
  for (size_t i = 0; i < MatcherType_COUNT; ++i)
  {
    m_matchers[i]->init();
    if (d_matcherType.getValue().getSelectedId() != i)
      m_matchers[i]->toggleVisible(false);
    else
      m_matchers[i]->init();
  }

  bindInputData(&d_queryDescriptors);
  bindInputData(&d_trainDescriptors);

  // Debug data: purposely not binded implicitely
  addInput(&d_in2);
  addInput(&d_kptsL);
  addInput(&d_kptsR);

  bindInputData(&d_mask);
  addOutput(&d_matches);
  setDirtyValue();
  ImageFilter::init();
}
void DescriptorMatcher::update()
{
  std::cout << getName() << std::endl;
  ImageFilter::update();

  sofa::helper::SVector<sofa::helper::SVector<common::cvDMatch> >* vec =
      d_matches.beginWriteOnly();
  vec->clear();
  if (!m_matches.empty() && !m_matches[0].empty())
  {
    for (std::vector<cv::DMatch>& matchVec : m_matches)
    {
      vec->push_back(helper::SVector<common::cvDMatch>());
      for (cv::DMatch& match : matchVec)
        vec->back().push_back(common::cvDMatch(match));
    }
  }
  d_matches.endEdit();
  d_matches.setDirtyOutputs();
  std::cout << "end" << getName() << std::endl;
}

void DescriptorMatcher::applyFilter(const cv::Mat& in, cv::Mat& out, bool debug)
{
  if (!f_listening.getValue()) return;

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
    m_matchers[m]->knnMatch(d_queryDescriptors.getValue(),
                            d_trainDescriptors.getValue(), m_matches,
                            d_k.getValue(), d_mask.getValue());
  else if (d_matchingAlgo.getValue().getSelectedId() == RADIUS_MATCH)
    m_matchers[m]->radiusMatch(d_queryDescriptors.getValue(),
                               d_trainDescriptors.getValue(), m_matches,
                               d_maxDistance.getValue(), d_mask.getValue());

  if (d_displayDebugWindow.getValue())
  {
    if (d_in2.isSet() && d_kptsL.isSet() && d_kptsR.isSet())
    {
      std::vector<cv::KeyPoint> kpL, kpR;
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_kptsL.getValue().data());
      kpL.assign(arr, arr + d_kptsL.getValue().size());
      arr = dynamic_cast<const cv::KeyPoint*>(d_kptsR.getValue().data());
      kpR.assign(arr, arr + d_kptsR.getValue().size());

      cv::drawMatches(in, kpL, d_in2.getValue(), kpR, m_matches, out);
    }
  }
}

void DescriptorMatcher::reinit()
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
  ImageFilter::reinit();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
