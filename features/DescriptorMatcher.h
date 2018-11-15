#ifndef SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H
#define SOFA_OR_PROCESSOR_DESCRIPTORMATCHER_H

#include "ImageProcessingPlugin.h"
#include "Matchers.h"

#include <SofaCV/SofaCV.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace features
{
class SOFA_IMAGEPROCESSING_API DescriptorMatcher : public ImageFilter
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
  virtual ~DescriptorMatcher() override;

  virtual void init() override;
  virtual void doUpdate() override;
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug) override;

  sofa::Data<sofa::helper::OptionsGroup> d_matcherType;
  sofa::Data<sofa::helper::OptionsGroup> d_matchingAlgo;
  sofa::Data<int> d_k;
  sofa::Data<float> d_maxDistance;
  sofa::Data<cvMat> d_mask;

  sofa::Data<cvMat> d_queryDescriptors;
  sofa::Data<cvMat> d_trainDescriptors;

  sofa::Data<cvMat> d_in2;
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_kptsL;
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_kptsR;

  sofa::Data<sofa::helper::SVector<sofa::helper::SVector<cvDMatch> > >
      d_matches;

  void match(const cvMat& queryDescriptors,
             std::vector<cv::DMatch>& matches);
  void knnMatch(const cvMat& queryDescriptors,
                std::vector<std::vector<cv::DMatch> >& matches, int k);
  void radiusMatch(const cvMat& queryDescriptors,
                   std::vector<std::vector<cv::DMatch> >& matches,
                   float maxDistance);

 protected:
  void matcherTypeChanged();

 private:
  BaseMatcher* m_matchers[MatcherType_COUNT];
  std::vector<std::vector<cv::DMatch> > m_matches;
};

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_DESCRIPTORMATCHER_H
