#ifndef SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H
#define SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H

#include "Matchers.h"
#include "core/ImageFilter.h"

#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvDMatch.h>
#include <SofaORCommon/cvMat.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
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

 public:
  DescriptorMatcher();
  virtual ~DescriptorMatcher();

  void init();
  void update();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug);
  void reinit();

  Data<sofa::helper::OptionsGroup> d_matcherType;
  Data<sofa::helper::OptionsGroup> d_matchingAlgo;
  Data<int> d_k;
  Data<float> d_maxDistance;
  Data<common::cvMat> d_mask;

  Data<common::cvMat> d_queryDescriptors;
  Data<common::cvMat> d_trainDescriptors;

  Data<common::cvMat> d_in2;
  Data<helper::vector<common::cvKeypoint> > d_kptsL;
  Data<helper::vector<common::cvKeypoint> > d_kptsR;

  Data<helper::SVector<helper::SVector<common::cvDMatch> > > d_matches;


  void match(const common::cvMat& queryDescriptors,
             std::vector<cv::DMatch>& matches);
  void knnMatch(const common::cvMat& queryDescriptors,
                std::vector<std::vector<cv::DMatch> >& matches, int k);
  void radiusMatch(const common::cvMat& queryDescriptors,
                   std::vector<std::vector<cv::DMatch> >& matches,
                   float maxDistance);

 private:
  BaseMatcher* m_matchers[MatcherType_COUNT];
  std::vector<std::vector<cv::DMatch> > m_matches;

};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H
