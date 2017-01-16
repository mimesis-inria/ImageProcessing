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
    : d_matcherType(initData(&d_matcherType, "matcher",
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

      d_queryDescriptors(initData(&d_queryDescriptors, "queryDescriptors",
                                  "Query set of descriptors")),
      d_trainDescriptors(initData(&d_trainDescriptors, "trainDescriptors",
                                  "Train set of descriptors")),
      d_matches(initData(&d_matches, "matches", "output array of matches", true,
                         true))
{
  f_listening.setValue(true);

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

  addInput(&d_queryDescriptors);
  addInput(&d_trainDescriptors);
  addOutput(&d_matches);
  setDirtyValue();
}
void DescriptorMatcher::update()
{
  if (!f_listening.getValue()) return;

  std::cout << getName() << std::endl;
  updateAllInputsIfDirty();
  cleanDirty();

  unsigned m = d_matcherType.getValue().getSelectedId();
  std::vector<std::vector<cv::DMatch> > matches;
  if (d_matchingAlgo.getValue().getSelectedId() == STANDARD_MATCH)
    m_matchers[m]->knnMatch(d_queryDescriptors.getValue(),
                        d_trainDescriptors.getValue(), matches, 1, *d_mask.beginEdit());
  else if (d_matchingAlgo.getValue().getSelectedId() == KNN_MATCH)
    m_matchers[m]->knnMatch(d_queryDescriptors.getValue(),
                        d_trainDescriptors.getValue(), matches, d_k.getValue(), *d_mask.beginEdit());
  else if (d_matchingAlgo.getValue().getSelectedId() == RADIUS_MATCH)
    m_matchers[m]->radiusMatch(d_queryDescriptors.getValue(),
                           d_trainDescriptors.getValue(), matches,
                           d_maxDistance.getValue(), *d_mask.beginEdit());
  d_mask.endEdit();

  sofa::helper::vector<sofa::helper::vector<common::cvDMatch> >* vec =
      d_matches.beginEdit();
  vec->clear();
  for (std::vector<cv::DMatch>& matchVec : matches)
  {
    vec->push_back(helper::vector<common::cvDMatch>());
    for (cv::DMatch& match : matchVec)
      vec->back().push_back(common::cvDMatch(match));
  }
  d_matches.endEdit();
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
}

void DescriptorMatcher::handleEvent(sofa::core::objectmodel::Event* e)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(e)) this->update();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
