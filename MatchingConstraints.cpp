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
    : ImageFilter(),
      d_useEpipolarFilter(
          initData(&d_useEpipolarFilter, false, "epipolarFilter",
                   "set to true to enable epipolar contrstraint filtering")),
      d_epipolarThreshold(
          initData(&d_epipolarThreshold, 5, "epipolarThreshold",
                   "in px, maximum distance to the epipolar line")),
      d_extrinsics(initData(
          &d_extrinsics, "extrinsics",
          "stereo camera's extrinsic parameters containing the "
          "Fundamental matrix to use to compute the epipolar "
          "lines (usually provided by CalibLoader, or computed with 2 frame's "
          "homography'). F can also be directly provided through "
          "the attribute \"F\"")),
      d_F(initData(
          &d_F, "F",
          "camera's Fundamental matrix to use to compute the epipolar "
          "lines. can also be provided through the attribute \"extrinsics\"")),
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
          &d_keypointsL_in, "keypointsL",
          "input keypoints left (usually from FeatureDetector)", true, true)),
      d_keypointsR_in(initData(
          &d_keypointsR_in, "keypointsR",
          "input keypoints right (usually from FeatureDetector)", true, true)),
      d_descriptorsL_in(initData(
          &d_descriptorsL_in, "descriptorsL",
          "input left descriptors (usually from FeatureDetector)", true, true)),
      d_descriptorsR_in(
          initData(&d_descriptorsR_in, "descriptorsR",
                   "input right descriptors  (usually from FeatureDetector)",
                   true, true)),
      d_matches_in(initData(&d_matches_in, "input_matches",
                            "feature matches (usually from DescriptorMatcher)",
                            true, true)),
      d_matches_out(initData(&d_matches_out, "output_matches",
                             "output matches optional usage, as keypoints and "
                             "descriptors are already paired in their "
                             "respective vectors",
                             true, true)),
      d_outliers_out(
          initData(&d_outliers_out, "outliers", "output vector of outliers")),
      d_keypointsL_out(initData(&d_keypointsL_out, "out_keypointsL",
                                "left keypoints", true, true)),
      d_keypointsR_out(initData(&d_keypointsR_out, "out_keypointsR",
                                "right keypoints", true, true)),
      d_descriptorsL_out(initData(&d_descriptorsL_out, "out_descriptorsL",
                                  "left descriptors", true, true)),
      d_descriptorsR_out(initData(&d_descriptorsR_out, "out_descriptorsR",
                                  "right descriptors", true, true)),
      d_epilinesL(initData(&d_epilinesL, "epilinesL",
                           "left epipolar lines for inliers", true, true)),
      d_epilinesR(initData(&d_epilinesR, "epilinesR",
                           "right epipolar lines for inliers", true, true)),
      d_epidistL(initData(&d_epidistL, "epidistL",
                          "left epipolar distances for inlier features", true,
                          true)),
      d_epidistR(initData(&d_epidistR, "epidistR",
                          "right epipolar distances for inlier features", true,
                          true)),
      d_mdfDistances(initData(&d_mdfDistances, "mdf_scores",
                              "mdf score for each inlier", true, true)),
      d_mdfMaxDist(
          initData(&d_mdfMaxDist, "mdf_maxdist",
                   "maximum distance for all features (including outliers)",
                   true, true)),
      d_knnLambdas(initData(&d_knnLambdas, "knn_scores",
                            "KNN score for each inlier", true, true))

{
  f_listening.setValue(true);
  m_outputImage = false;
}

MatchingConstraints::~MatchingConstraints() {}
void MatchingConstraints::init()
{
  addInput(&d_keypointsL_in);
  addInput(&d_keypointsR_in);
  addInput(&d_descriptorsL_in);
  addInput(&d_descriptorsR_in);
  addInput(&d_matches_in);

  addInput(&d_extrinsics);
  addInput(&d_F);

  addOutput(&d_keypointsL_out);
  addOutput(&d_keypointsR_out);
  addOutput(&d_descriptorsL_out);
  addOutput(&d_descriptorsR_out);
  addOutput(&d_outliers_out);

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
    d_extrinsics.setDisplayed(false);
  }
  if (!d_useMDFilter.getValue())
  {
    d_mdfRadius.setDisplayed(false);
  }
  if (!d_useKNNFilter.getValue())
  {
    d_knnLambda.setDisplayed(false);
  }
  setDirtyValue();

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
    cv::computeCorrespondEpilines(ptsL, 1, d_F.getValue(), m_epilinesL);
    cv::computeCorrespondEpilines(ptsR, 2, d_F.getValue(), m_epilinesR);
  }
  else if (!d_extrinsics.getValue().F.empty())
  {
    cv::Mat_<double> F;
    common::matrix::sofaMat2cvMat(d_extrinsics.getValue().F, F);
    cv::computeCorrespondEpilines(ptsL, 1, F, m_epilinesL);
    cv::computeCorrespondEpilines(ptsR, 2, F, m_epilinesR);
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
  // All precomputations for the filters, only done once per new batch of
  // inputs
  updateAllInputsIfDirty();
  cleanDirty();
  if (!f_listening.getValue()) return;

  if (d_keypointsR_in.getValue().size() != d_descriptorsR_in.getValue().rows)
  {
    std::cout << "WTF????" << std::endl;
    return;
  }
  m_maxDist = .0;

  m_matches_out.clear();
  m_kptsL.clear();
  m_kptsR.clear();
  m_kptsL.reserve(d_matches_in.getValue().size());
  m_kptsR.reserve(d_matches_in.getValue().size());
  m_matches_out.reserve(d_matches_in.getValue().size());

  m_descL = cv::Mat(int(d_matches_in.getValue().size()),
                    d_descriptorsL_in.getValue().cols,
                    d_descriptorsL_in.getValue().type());
  m_descR = cv::Mat(int(d_matches_in.getValue().size()),
                    d_descriptorsL_in.getValue().cols,
                    d_descriptorsL_in.getValue().type());
  const helper::vector<common::cvKeypoint>& PointsL =
      d_keypointsL_in.getValue();
  const helper::vector<common::cvKeypoint>& PointsR =
      d_keypointsR_in.getValue();
  const cv::Mat& DescriptorsLeft = d_descriptorsL_in.getValue();
  const cv::Mat& DescriptorsRight = d_descriptorsR_in.getValue();

  const helper::vector<common::cvDMatch>* ptr = d_matches_in.getValue().data();
  int l_index, r_index;
  std::vector<cv::DMatch> dm(1, cv::DMatch());
  for (size_t i = 0; i < d_matches_in.getValue().size(); ++i)
  {
    float d = (*ptr)[0].distance;
    if (d > m_maxDist) m_maxDist = d;
    l_index = (*ptr)[0].queryIdx;
    r_index = (*ptr)[0].trainIdx;

    dm.push_back((*ptr)[0]);
    dm.back().queryIdx = i;
    dm.back().trainIdx = i;
    m_matches_out.push_back(dm);
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
  setDirtyOutputs();
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
  helper::SVector<helper::SVector<common::cvDMatch> >& out_matches = *d_matches_out.beginWriteOnly();
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
    std::cout << "WTF????" << std::endl;
    return;
  }
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
    size_t inliersIdx = i - outliers.size();

    dm[0] = in_matches[i][0];
    dm[0].trainIdx = i;
    dm[0].queryIdx = i;
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
  d_keypointsL_out.endEdit();
  d_keypointsR_out.endEdit();
  d_descriptorsL_out.endEdit();
  d_descriptorsR_out.endEdit();

  d_epidistL.endEdit();
  d_epidistR.endEdit();
  d_epilinesL.endEdit();
  d_epilinesR.endEdit();

  d_mdfDistances.endEdit();

  d_knnLambdas.endEdit();
}

void MatchingConstraints::reinit() { ImageFilter::reinit(); }
}  // namespace processor
}  // namespace OR
}  // namespace sofa
