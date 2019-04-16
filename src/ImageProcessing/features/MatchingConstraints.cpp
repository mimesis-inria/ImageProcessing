#include "MatchingConstraints.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
namespace sofacv
{
namespace features
{
SOFA_DECL_CLASS(MatchingConstraints)

int MatchingConstraintsClass =
    sofa::core::RegisterObject("Constraint filtering for opencv matcher")
        .add<MatchingConstraints>();

MatchingConstraints::MatchingConstraints()
    : l_cam(initLink("cam",
                     "StereoSettings holding the fundamental matrix needed for "
                     "the epipolar constraint filtering")),
      d_useEpipolarFilter(
          initData(&d_useEpipolarFilter, false, "epipolarFilter",
                   "set to true to enable epipolar contrstraint filtering")),
      d_epipolarThreshold(
          initData(&d_epipolarThreshold, 5, "epipolarThreshold",
                   "in px, maximum distance to the epipolar line")),
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
          "input keypoints left (usually from FeatureDetector)", true, true)),
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
                                "left keypoints", true, true)),
      d_keypointsR_out(initData(&d_keypointsR_out, "keypoints2_out",
                                "right keypoints", false, true)),
      d_descriptorsL_out(initData(&d_descriptorsL_out, "descriptors1_out",
                                  "left descriptors", false, true)),
      d_descriptorsR_out(initData(&d_descriptorsR_out, "descriptors2_out",
                                  "right descriptors", false, true))
{
  addAlias(&d_outliers_out, "outliers_out");

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

  addOutput(&d_matches_out);
  addOutput(&d_outliers_out);
  addOutput(&d_keypointsL_out);
  addOutput(&d_keypointsR_out);
  addOutput(&d_descriptorsL_out);
  addOutput(&d_descriptorsR_out);

  if (!d_useEpipolarFilter.getValue())
  {
    d_epipolarThreshold.setDisplayed(false);
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

  if (!l_cam.get())
    msg_warning(getName() + "::init()")
        << "No Stereo camera settings link set. "
           "Please use attribute 'cam' if you plan on using the epipolar "
           "constraint filtering";

  ImageFilter::init();
}

bool MatchingConstraints::computeEpipolarLines()
{
  m_epilines.clear();

  if (d_keypointsL_in.getValue().empty())
  {
    std::cout << "No Keypoints to compute epipolar lines for!" << std::endl;
    return false;
  }
  std::vector<cv::Point2f> ptsL;
  ptsL.reserve(d_keypointsL_in.getValue().size());

  for (auto kp : d_keypointsL_in.getValue())
  {
    ptsL.push_back(kp.pt);
  }

  if (!l_cam.get() || !l_cam->getFundamentalMatrix().empty())
  {
    cv::Mat F;
    matrix::sofaMat2cvMat(l_cam->getFundamentalMatrix(), F);
    cv::computeCorrespondEpilines(ptsL, 1, F, m_epilines);
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

void MatchingConstraints::doUpdate()
{
  std::cout << getName() << std::endl;
  // All precomputations for the filters, only done once per new batch of
  // inputs

  // Making sure we have keypoints, and that we have an even number of keypoints
  // and descriptors
  if (d_matches_in.getValue().empty())
  {
    msg_error(getName() + "::update()") << "Error: No match to filter!";
    return;
  }
  if (d_keypointsR_in.getValue().size() !=
      size_t(d_descriptorsR_in.getValue().rows))
  {
    msg_error(getName() + "::update()")
        << "Error: number of Right keypoints and descriptors differ!";
    return;
  }
  if (d_keypointsL_in.getValue().size() !=
      size_t(d_descriptorsL_in.getValue().rows))
  {
    msg_error(getName() + "::update()")
        << "Error: number of Left keypoints and descriptors differ!";
    return;
  }

  m_maxDist = .0;

  // copy inputs into Opencv data structures:
  std::vector<std::vector<cv::DMatch> > matches;
  for (auto matchVector : d_matches_in.getValue())
  {
    std::vector<cv::DMatch> mvec;
    for (auto match : matchVector) mvec.push_back(match);
    matches.push_back(mvec);
  }
  const sofa::helper::vector<cvKeypoint>& PointsR = d_keypointsR_in.getValue();

  /// Fill matchVector with all the necessary info:
  m_matchVector.clear();

  /// Precompute all epipolar lines for left keypoints
  cv::Mat F;
  matrix::sofaMat2cvMat(l_cam->getFundamentalMatrix(), F);
  if (d_useEpipolarFilter.getValue()) computeEpipolarLines();

  for (auto matchVector : matches)
  {
    /// retrieve maximum value for MDF
    if (matchVector[0].distance > m_maxDist)
      m_maxDist = matchVector[0].distance;

    MatchVector mv;
    mv.idxL = unsigned(matchVector[0].queryIdx);
    for (const cv::DMatch& match : matchVector)
    {
      /// IF you want to filter distance according to EVERY match, including 2nd
      /// degree matches & more, uncomment the following line:
      //      if (match.distance > m_maxDist) m_maxDist = match.distance;

      /// Storing, for each right match of a left point, its distance, index,
      /// and
      /// distance to the epipolar line
      MatchStruct ms;
      ms.distance = unsigned(match.distance);
      ms.idxR = unsigned(match.trainIdx);
      ms.distanceToEpiline =
          unsigned(distancePointLine(PointsR[ms.idxR].pt, m_epilines[mv.idxL]));

      mv.matches.push_back(ms);
    }
    m_matchVector.push_back(mv);
  }

  ImageFilter::update();

  d_outliers_out.setValue(m_outliers_out);
  d_matches_out.setValue(m_matches);
  d_keypointsL_out.setValue(m_kpL);
  d_keypointsR_out.setValue(m_kpR);
  d_descriptorsL_out.setValue(m_descL);
  d_descriptorsR_out.setValue(m_descR);
  std::cout << "final number of kpL: " << d_keypointsL_out.getValue().size()
            << std::endl;
}
bool MatchingConstraints::EpipolarConstraintFilter(unsigned& filteredByEpipolar,
                                                   unsigned i, MatchVector& ms,
                                                   double epiDist)
{
  std::vector<MatchStruct> filteredMatches;
  for (const MatchStruct& match : ms.matches)
  {
    if (epiDist >= match.distanceToEpiline) filteredMatches.push_back(match);
  }
  if (filteredMatches.empty())
  {
    m_outliers_out.push_back(i);
    filteredByEpipolar++;
    return false;
  }
  ms.matches = filteredMatches;
  return true;
}

bool MatchingConstraints::KNearestNeighborFilter(unsigned i, double lambda,
                                                 MatchVector& ms,
                                                 unsigned& filteredByKNN)
{
  if (ms.matches[1].distance / ms.matches[0].distance < lambda)
  {
    m_outliers_out.push_back(i);
    filteredByKNN++;
    return false;
  }
  return true;
}

bool MatchingConstraints::MinimalDistanceFilter(MatchVector& ms,
                                                unsigned& filteredByMDF,
                                                unsigned i, float mdfDist)
{
  std::vector<MatchStruct> filteredMatches;
  for (const MatchStruct& match : ms.matches)
  {
    float mdfVal = match.distance / m_maxDist;
    if (mdfDist > mdfVal) filteredMatches.push_back(match);
  }
  if (filteredMatches.empty())
  {
    m_outliers_out.push_back(i);
    filteredByMDF++;
    return false;
  }
  ms.matches = filteredMatches;
  return true;
}

void MatchingConstraints::PushInlier(
    const cvMat& descL, unsigned i,
    const sofa::helper::vector<cvKeypoint>& PointsL, const cvMat& descR,
    MatchVector& ms, const sofa::helper::vector<cvKeypoint>& PointsR)
{
  unsigned long inliersIdx = i - m_outliers_out.size();
  m_matches.push_back(
      cvDMatch(int(ms.idxL), int(ms.matches[0].idxR), ms.matches[0].distance));
  m_kpL.push_back(PointsL[ms.idxL]);
  m_kpR.push_back(PointsR[ms.matches[0].idxR]);

  descL.row(int(ms.idxL)).copyTo(m_descL.row(int(inliersIdx)));
  descR.row(int(ms.matches[0].idxR)).copyTo(m_descR.row(int(inliersIdx)));
}

void MatchingConstraints::ClearOutputVectors()
{
  m_matches.clear();
  m_kpL.clear();
  m_kpR.clear();
  m_outliers_out.clear();
  m_descL = cvMat();
  m_descR = cvMat();

  // preallocating space for structures based on the number of keypoints on the
  // reference image (Left)
  m_matches.reserve(d_keypointsL_in.getValue().size());
  m_kpL.reserve(d_keypointsL_in.getValue().size());
  m_kpR.reserve(d_keypointsL_in.getValue().size());
  m_outliers_out.reserve(d_keypointsL_in.getValue().size());
  m_descL = cvMat(int(d_keypointsL_in.getValue().size()),
                  d_descriptorsL_in.getValue().cols,
                  d_descriptorsL_in.getValue().type());
  m_descR = cvMat(int(d_keypointsL_in.getValue().size()),
                  d_descriptorsL_in.getValue().cols,
                  d_descriptorsL_in.getValue().type());
}

void MatchingConstraints::applyFilter(const cv::Mat& in, cv::Mat& out, bool)
{
  // Actual application of filters, filling outputs
  bool epipolar = d_useEpipolarFilter.getValue();
  double epiDist = d_epipolarThreshold.getValue();

  bool mdf = d_useMDFilter.getValue();
  float mdfDist = d_mdfRadius.getValue();

  bool knn = d_useKNNFilter.getValue();
  double lambda = double(d_knnLambda.getValue());

  // ensuring that initial output matches, keypoints and descriptors are empty
  ClearOutputVectors();

  /// Prepare the output matrix to display the colored points
  if (d_outputImage.getValue())
  {
    in.copyTo(out);
    if (in.depth() == CV_32F)
    {
      std::cout << "converting to 8bit" << std::endl;
      out.convertTo(out, 0, 255.0);
    }
    if (out.channels() == 1)
    {
      std::cout << "converting from grayscale to BGR" << std::endl;
      cv::cvtColor(out, out, CV_GRAY2BGR);
    }
  }

  const sofa::helper::vector<cvKeypoint>& PointsL = d_keypointsL_in.getValue();
  const sofa::helper::vector<cvKeypoint>& PointsR = d_keypointsR_in.getValue();

  const cvMat& descL = d_descriptorsL_in.getValue();
  const cvMat& descR = d_descriptorsR_in.getValue();

  unsigned filteredByEpipolar = 0;
  unsigned filteredByKNN = 0;
  unsigned filteredByMDF = 0;

  /// Start filtering points out
  for (unsigned i = 0; i < m_matchVector.size(); ++i)
  {
    MatchVector& ms = m_matchVector[i];

    // Epipolar constraint filtering
    if (epipolar &&
        !EpipolarConstraintFilter(filteredByEpipolar, i, ms, epiDist))
      continue;

    // KNN constraint filtering (k2 / k1 < lambda)
    if (knn && ms.matches.size() > 1 &&
        !KNearestNeighborFilter(i, lambda, ms, filteredByKNN))
      continue;

    // Minimal distance filtering
    if (mdf && !MinimalDistanceFilter(ms, filteredByMDF, i, mdfDist)) continue;

    /// store Inliers into output vectors
    PushInlier(descL, i, PointsL, descR, ms, PointsR);

    // Draw inlier on output image
    if (d_outputImage.getValue())
      cv::circle(out, m_kpL.back().pt, 3, cv::Scalar(0, 255, 0));
  }

  std::cout << "inliers: " << m_kpL.size() << std::endl;

  std::cout << filteredByEpipolar << " points filtered by Epipolar Constraint"
            << std::endl;
  std::cout << filteredByKNN << " points filtered by K-Nearest-Neighbor"
            << std::endl;
  std::cout << filteredByMDF << " points filtered by Minimal Distance"
            << std::endl;
  std::cout << m_outliers_out.size() << " outliers in total" << std::endl;
}

}  // namespace features
}  // namespace sofacv
