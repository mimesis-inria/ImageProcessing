#ifndef SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
#define SOFA_OR_PROCESSOR_FEATUREDETECTOR_H

#include "initplugin.h"

#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvDMatch.h>
#include <SofaORCommon/cvMat.h>

#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class DescriptorMatcher : public core::DataEngine
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
  SOFA_CLASS(DescriptorMatcher, core::DataEngine);

 public:
  DescriptorMatcher();
  virtual ~DescriptorMatcher();

  void init();
  void update();
  void reinit();

  Data<sofa::helper::OptionsGroup> d_matcherType;
  Data<sofa::helper::OptionsGroup> d_matchingAlgo;
  Data<int> d_k;
  Data<float> d_maxDistance;

  Data<common::cvMat> d_queryDescriptors;
  Data<common::cvMat> d_trainDescriptors;

  Data<helper::vector<helper::vector<common::cvDMatch> > > d_matches;

  void match(const common::cvMat& queryDescriptors,
             std::vector<cv::DMatch>& matches);
  void knnMatch(const common::cvMat& queryDescriptors,
                std::vector<std::vector<cv::DMatch> >& matches, int k);
  void radiusMatch(const common::cvMat& queryDescriptors,
                   std::vector<std::vector<cv::DMatch> >& matches,
                   float maxDistance);

  void handleEvent(sofa::core::objectmodel::Event* event);

 private:
  cv::DescriptorMatcher* m_matcher;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_FEATUREDETECTOR_H
