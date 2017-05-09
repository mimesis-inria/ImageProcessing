#include "MatchingConstraints.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(MatchingConstraints)

int MatchingConstraintsClass =
    core::RegisterObject("Constraint filtering for opencv matcher")
        .add<MatchingConstraints>();

MatchingConstraints::MatchingConstraints()
    : ImageFilter(false),
      d_useEpipolarFilter(
          initData(&d_useEpipolarFilter, false, "epipolarFilter",
                   "set to true to enable epipolar contrstraint filtering")),
      d_epipolarThreshold(
          initData(&d_epipolarThreshold, 5, "epipolarThreshold",
                   "in px, maximum distance to the epipolar line")),
      d_F(initData(&d_F, "F",
                   "camera's Fundamental matrix to use to compute the epipolar "
                   "lines.")),
      d_useMDFilter(initData(
          &d_useMDFilter, true, "MDFilter",
          "set to true to enable minimal distance filtering constraint")),
      d_mdfRadius(initData(&d_mdfRadius, 0.99f, "mdfThreshold",
                           "minimal distance filtering threshold")),
      d_useKNNFilter(initData(
          &d_useKNNFilter, true, "KNNFilter",
          "set to true to enable k-nearest neighbor filtering constraint")),
      d_knnLambda(initData(&d_knnLambda, 1.4f, "KNNThreshold",
                           "lambda value for KNNFilter (d2 / d1 < lambda)")),
      d_keypointsL_in(initData(
          &d_keypointsL_in, "keypoints1",
          "input keypoints left (usually from FeatureDetector)", false, true)),
      d_keypointsR_in(initData(
          &d_keypointsR_in, "keypoints2",
          "input keypoints right (usually from FeatureDetector)", false, true)),
      d_descriptorsL_in(
          initData(&d_descriptorsL_in, "descriptors1",
                   "input left descriptors (usually from FeatureDetector)",
                   false, true)),
      d_descriptorsR_in(
          initData(&d_descriptorsR_in, "descriptors2",
                   "input right descriptors  (usually from FeatureDetector)",
                   false, true)),
      d_matches_in(initData(&d_matches_in, "matches",
                            "feature matches (usually from DescriptorMatcher)",
                            false, true)),
      d_matches_out(initData(&d_matches_out, "matches_out",
                             "output matches optional usage, as keypoints and "
                             "descriptors are already paired in their "
                             "respective vectors",
                             false, true)),
      d_outliers_out(initData(&d_outliers_out, "outliers",
                              "output vector of outliers", false)),
      d_keypointsL_out(initData(&d_keypointsL_out, "keypoints1_out",
                                "left keypoints", false, true)),
      d_keypointsR_out(initData(&d_keypointsR_out, "keypoints2_out",
                                "right keypoints", false, true)),
      d_descriptorsL_out(initData(&d_descriptorsL_out, "descriptors1_out",
                                  "left descriptors", false, true)),
      d_descriptorsR_out(initData(&d_descriptorsR_out, "descriptors2_out",
                                  "right descriptors", false, true)),
      d_epilinesL(initData(&d_epilinesL, "epilines1",
                           "left epipolar lines for inliers", false, true)),
      d_epilinesR(initData(&d_epilinesR, "epilines2",
                           "right epipolar lines for inliers", false, true)),
      d_epidistL(initData(&d_epidistL, "epidist1",
                          "left epipolar distances for inlier features", false,
                          true)),
      d_epidistR(initData(&d_epidistR, "epidist2",
                          "right epipolar distances for inlier features", false,
                          true)),
      d_mdfDistances(initData(&d_mdfDistances, "mdfScores",
                              "mdf score for each inlier", false, true)),
      d_mdfMaxDist(
          initData(&d_mdfMaxDist, "mdfMaxDist",
                   "maximum distance for all features (including outliers)",
                   true, true)),
      d_knnLambdas(initData(&d_knnLambdas, "knnScores",
                            "KNN score for each inlier", false, true))

{
  addAlias(&d_outliers_out, "outliers_out");
  addAlias(&d_epilinesL, "epilines1_out");
  addAlias(&d_epilinesR, "epilines2_out");
  addAlias(&d_epidistL, "epidist1_out");
  addAlias(&d_epidistR, "epidist2_out");

  addAlias(&d_mdfDistances, "mdfScores_out");
  addAlias(&d_mdfMaxDist, "mdfMaxDist_out");
  addAlias(&d_knnLambdas, "knnScores_out");

  f_listening.setValue(true);
}

MatchingConstraints::~MatchingConstraints() {}
void MatchingConstraints::init()
{
  addInput(&d_keypointsL_in);
  addInput(&d_keypointsR_in);
  addInput(&d_descriptorsL_in);
  addInput(&d_descriptorsR_in);
  addInput(&d_matches_in);

  addInput(&d_F);

  addOutput(&d_matches_out);
  addOutput(&d_outliers_out);
  addOutput(&d_keypointsL_out);
  addOutput(&d_keypointsR_out);
  addOutput(&d_descriptorsL_out);
  addOutput(&d_descriptorsR_out);

  addOutput(&d_epidistL);
  addOutput(&d_epidistR);
  addOutput(&d_epilinesL);
  addOutput(&d_epilinesR);

  addOutput(&d_mdfDistances);
  addOutput(&d_mdfMaxDist);

  addOutput(&d_knnLambdas);

  if (!d_useEpipolarFilter.getValue())
  {
    d_epipolarThreshold.setDisplayed(false);
    d_F.setDisplayed(false);
  }
  if (!d_useMDFilter.getValue())
  {
    d_mdfRadius.setDisplayed(false);
  }
  if (!d_useKNNFilter.getValue())
  {
    d_knnLambda.setDisplayed(false);
  }
  registerData(&d_useEpipolarFilter);
  registerData(&d_epipolarThreshold, 0, 255, 1);
  registerData(&d_useKNNFilter);
  registerData(&d_knnLambda, 0.0f, 4.0f, 0.01f);
  registerData(&d_useMDFilter);
  registerData(&d_mdfRadius, 0.0f, 1.0f, 0.001f);

  ImageFilter::init();
}

bool MatchingConstraints::computeEpipolarLines()
{
  m_epilinesL.clear();
  m_epilinesR.clear();
  m_epidistanceL.clear();
  m_epidistanceR.clear();

  if (m_kptsL.empty() || m_kptsR.empty()) return false;

  std::vector<cv::Point2f> ptsL, ptsR;
  ptsL.reserve(m_kptsL.size());
  ptsR.reserve(m_kptsR.size());

  for (size_t i = 0; i < m_kptsL.size(); ++i)
  {
    ptsL.push_back(m_kptsL[i].pt);
    ptsR.push_back(m_kptsR[i].pt);
  }

  if (!d_F.getValue().empty())
  {
    cv::Mat_<double> f;
    common::matrix::sofaMat2cvMat(d_F.getValue(), f);
    cv::computeCorrespondEpilines(ptsL, 1, f, m_epilinesL);
    cv::computeCorrespondEpilines(ptsR, 2, f, m_epilinesR);
  }
  else
  {
    msg_error("MatchingConstraints::computeEpipolarLines()")
        << "Fundamental matrix not provided";
    return false;
  }
  return true;
}

float distancePointLine(const cv::Point2f point, const cv::Vec3f& line)
{
  // Line is given as a*x + b*y + c = 0
  return fabsf(line(0) * point.x + line(1) * point.y + line(2)) /
         std::sqrt(line(0) * line(0) + line(1) * line(1));
}

void MatchingConstraints::computeEpipolarDistances()
{
  for (size_t i = 0; i < m_kptsL.size(); ++i)
  {
    m_epidistanceL.push_back(distancePointLine(m_kptsL[i].pt, m_epilinesR[i]));
    m_epidistanceR.push_back(distancePointLine(m_kptsR[i].pt, m_epilinesL[i]));
  }
}

void MatchingConstraints::update()
{
  std::cout << getName() << std::endl;
  // All precomputations for the filters, only done once per new batch of
  // inputs
  if (d_keypointsR_in.getValue().size() !=
      size_t(d_descriptorsR_in.getValue().rows))
  {
    std::cout << "-- WTF???? ##KPTS != DESC##" << std::endl;
    return;
  }
  m_maxDist = .0;

  const helper::SVector<helper::SVector<common::cvDMatch> >& matches =
      d_matches_in.getValue();
  m_matches_out.clear();
  m_kptsL.clear();
  m_kptsR.clear();
  m_kptsL.reserve(matches.size());
  m_kptsR.reserve(matches.size());
  m_matches_out.resize(matches.size(), std::vector<cv::DMatch>(2));

  for (size_t i = 0; i < matches.size(); ++i)
    for (size_t j = 0; j < 2; ++j) m_matches_out[i][j] = matches[i][j];
  m_descL = cv::Mat(int(matches.size()), d_descriptorsL_in.getValue().cols,
                    d_descriptorsL_in.getValue().type());
  m_descR = cv::Mat(int(matches.size()), d_descriptorsL_in.getValue().cols,
                    d_descriptorsL_in.getValue().type());
  const helper::vector<common::cvKeypoint>& PointsL =
      d_keypointsL_in.getValue();
  const helper::vector<common::cvKeypoint>& PointsR =
      d_keypointsR_in.getValue();
  const cv::Mat& DescriptorsLeft = d_descriptorsL_in.getValue();
  const cv::Mat& DescriptorsRight = d_descriptorsR_in.getValue();

  std::vector<cv::DMatch>* ptr = m_matches_out.data();
  int l_index, r_index;
  for (size_t i = 0; i < matches.size(); ++i)
  {
    float d = (*ptr)[0].distance;
    if (d > m_maxDist) m_maxDist = d;
    l_index = (*ptr)[0].queryIdx;
    r_index = (*ptr)[0].trainIdx;

    (*ptr)[0].queryIdx = i;
    (*ptr)[0].trainIdx = i;
    m_kptsL.push_back(PointsL[unsigned(l_index)]);
    m_kptsR.push_back(PointsR[unsigned(r_index)]);
    DescriptorsLeft.row(l_index).copyTo(m_descL.row(int(i)));
    DescriptorsRight.row(r_index).copyTo(m_descR.row(int(i)));
    m_mdfDistances.push_back(d);
    ptr++;
  }

  if (d_useEpipolarFilter.getValue())
    if (computeEpipolarLines()) computeEpipolarDistances();

  ImageFilter::update();
  std::cout << d_matches_out.getValue().size() << std::endl;
  std::cout << d_keypointsL_out.getValue().size() << std::endl;
  std::cout << "end" << getName() << std::endl;
}
void MatchingConstraints::applyFilter(const cv::Mat& in, cv::Mat& out, bool)
{
  // Actual application of filters, filling outputs
  bool epipolar = d_useEpipolarFilter.getValue();
  double epiDist = d_epipolarThreshold.getValue();

  bool mdf = d_useMDFilter.getValue();
  float mdfDist = d_mdfRadius.getValue();

  bool knn = d_useKNNFilter.getValue();
  double lambda = d_knnLambda.getValue();

  helper::vector<size_t>& outliers = *d_outliers_out.beginWriteOnly();
  helper::SVector<helper::SVector<common::cvDMatch> >& out_matches =
      *d_matches_out.beginWriteOnly();
  sofa::helper::vector<common::cvKeypoint>& kptsL =
      *d_keypointsL_out.beginWriteOnly();
  sofa::helper::vector<common::cvKeypoint>& kptsR =
      *d_keypointsR_out.beginWriteOnly();
  common::cvMat& descL = *d_descriptorsL_out.beginWriteOnly();
  common::cvMat& descR = *d_descriptorsR_out.beginWriteOnly();

  helper::vector<float>& epidistL = *d_epidistL.beginWriteOnly();
  helper::vector<float>& epidistR = *d_epidistR.beginWriteOnly();
  helper::vector<defaulttype::Vec3f>& epilinesL = *d_epilinesL.beginWriteOnly();
  helper::vector<defaulttype::Vec3f>& epilinesR = *d_epilinesR.beginWriteOnly();

  helper::vector<float>& mdfDistances = *d_mdfDistances.beginWriteOnly();

  helper::vector<float>& knnLambdas = *d_knnLambdas.beginWriteOnly();

  // outputs initialization
  outliers.clear();
  outliers.reserve(d_matches_in.getValue().size());
  out_matches.resize(d_matches_in.getValue().size());
  kptsL.resize(d_matches_in.getValue().size());
  kptsR.resize(d_matches_in.getValue().size());

  descL = cv::Mat(int(d_matches_in.getValue().size()),
                  d_descriptorsL_in.getValue().cols,
                  d_descriptorsL_in.getValue().type());
  descR = cv::Mat(int(d_matches_in.getValue().size()),
                  d_descriptorsL_in.getValue().cols,
                  d_descriptorsL_in.getValue().type());

  if (epipolar)
  {
    epidistL.resize(d_matches_in.getValue().size());
    epidistR.resize(d_matches_in.getValue().size());
    epilinesL.resize(d_matches_in.getValue().size());
    epilinesR.resize(d_matches_in.getValue().size());
  }
  else
  {
    epidistL.clear();
    epidistR.clear();
    epilinesL.clear();
    epilinesR.clear();
  }
  if (mdf)
  {
    d_mdfMaxDist.setValue(m_maxDist);
    mdfDistances.resize(d_matches_in.getValue().size());
  }
  else
    mdfDistances.clear();
  if (knn)
    knnLambdas.resize(d_matches_in.getValue().size());
  else
    knnLambdas.clear();

  const helper::SVector<helper::SVector<common::cvDMatch> >& in_matches =
      d_matches_in.getValue();

  if (d_displayDebugWindow.getValue()) in.copyTo(out);
  if (m_kptsL.size() != in_matches.size())
  {
    std::cout << "-- WTF???? ##KPTS != MATCHES##" << std::endl;
    return;
  }
  size_t inliersIdx = 0;
  helper::SVector<common::cvDMatch> dm(1, common::cvDMatch());
  for (size_t i = 0; i < m_kptsL.size(); ++i)
  {
    // Epipolar constraint filtering
    if (epipolar &&
        (epiDist < m_epidistanceL[i] || epiDist < m_epidistanceR[i]))
    {
      outliers.push_back(i);
      continue;
    }

    // KNN constraint filtering
    float knnVal = -1;
    if (knn && in_matches[i].size() > 1)
    {
      knnVal = (in_matches[i][1].distance / in_matches[i][0].distance);
      if (knnVal < lambda)
      {
        outliers.push_back(i);
        continue;
      }
    }

    // Minimal distance filtering
    float mdfVal = (m_mdfDistances[i] / m_maxDist);
    if (mdf && (mdfDist < mdfVal))
    {
      outliers.push_back(i);
      continue;
    }

    // inliers
    // /!\ No push_back here, as vectors & matrices are not cleaned /!\ //
    inliersIdx = i - outliers.size();

    dm[0] = in_matches[i][0];
    dm[0].trainIdx = inliersIdx;
    dm[0].queryIdx = inliersIdx;
    out_matches[inliersIdx] = dm;
    kptsL[inliersIdx] = m_kptsL[i];
    kptsR[inliersIdx] = m_kptsR[i];
    m_descL.row(int(inliersIdx)).copyTo(descL.row(int(i)));
    m_descR.row(int(inliersIdx)).copyTo(descR.row(int(i)));

    if (d_displayDebugWindow.getValue())
      cv::circle(out, kptsL[inliersIdx].pt, 3, cv::Scalar(0, 255, 0));

    if (epipolar)
    {
      epidistL[inliersIdx] = m_epidistanceL[i];
      epidistR[inliersIdx] = m_epidistanceR[i];
      epilinesL[inliersIdx] = defaulttype::Vec3f(m_epilinesL[i].val);
      epilinesR[inliersIdx] = defaulttype::Vec3f(m_epilinesR[i].val);
    }
    if (mdf) mdfDistances[inliersIdx] = mdfVal;
    if (knn && in_matches[0].size() > 1) knnLambdas[inliersIdx] = knnVal;
  }

  d_outliers_out.endEdit();
  out_matches.resize(inliersIdx);
  d_matches_out.endEdit();
  kptsL.resize(inliersIdx);
  d_keypointsL_out.endEdit();
  kptsR.resize(inliersIdx);
  d_keypointsR_out.endEdit();
  descL.resize(inliersIdx);
  d_descriptorsL_out.endEdit();
  descR.resize(inliersIdx);
  d_descriptorsR_out.endEdit();

  epidistL.resize(inliersIdx);
  d_epidistL.endEdit();
  epidistR.resize(inliersIdx);
  d_epidistR.endEdit();
  epilinesL.resize(inliersIdx);
  d_epilinesL.endEdit();
  epilinesR.resize(inliersIdx);
  d_epilinesR.endEdit();

  mdfDistances.resize(inliersIdx);
  d_mdfDistances.endEdit();

  knnLambdas.resize(inliersIdx);
  d_knnLambdas.endEdit();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
