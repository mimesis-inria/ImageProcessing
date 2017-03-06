#include "FeatureTriangulator.h"
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(FeatureTriangulator)

int FeatureTriangulatorClass =
    core::RegisterObject(
        "component generating a 3D point cloud from two 2D keypoints list and "
        "their matches, and stereo camera parameters")
        .add<FeatureTriangulator>();

FeatureTriangulator::FeatureTriangulator()
    : d_rectify(initData(
          &d_rectify, false, "rectify",
          "if set to true, points will be rectified before triangulating")),
      d_R(initData(&d_R, "R",
                   "3x3 rotation matrix (extrinsic parameter of the camera)",
                   true, true)),
      d_T(initData(&d_T, "T",
                   "translation vector (extrinsic parameter of the camera)",
                   true, true)),
      d_cmL(initData(&d_cmL, "projMat1",
                     "projection matrix for the first camera", true, true)),
      d_cmR(initData(&d_cmR, "projMat2",
                     "projection matrix for the second camera", true, true)),
      d_dvL(initData(&d_dvL, "distCoefs1",
                     "distortion coefficients for the first camera", true,
                     true)),
      d_dvR(initData(&d_dvR, "distCoefs2",
                     "distortion coefficients for the second camera", true,
                     true)),
      d_keypointsL(initData(&d_keypointsL, "keypoints1",
                            "input vector of left keypoints", true, true)),
      d_keypointsR(initData(&d_keypointsR, "keypoints2",
                            "input vector of right keypoints", true, true)),
      d_matches(initData(&d_matches, "matches",
                         "input array of matches between the 2 vectors "
                         "(optional if keypoints are already sorted).",
                         true, true)),
      d_pointCloud(
          initData(&d_pointCloud, "positions", "output vector of 3D points")),
      d_pointCloudColors(initData(&d_pointCloudColors, "colors",
                                  "output vector of rgb point color")),
      d_img(initData(
          &d_img, "img",
          "image from which to extract point color (based on left keypoints"))
{
  f_listening.setValue(true);
  addAlias(&d_pointCloud, "positions_out");
  addAlias(&d_pointCloudColors, "colors_out");
}

FeatureTriangulator::~FeatureTriangulator() {}
void FeatureTriangulator::init()
{
  addInput(&d_R);
  addInput(&d_T);
  addInput(&d_cmL);
  addInput(&d_cmR);
  addInput(&d_dvL);
  addInput(&d_dvR);

  addInput(&d_matches);
  addInput(&d_keypointsL);
  addInput(&d_keypointsR);

  addInput(&d_img, true);

  addOutput(&d_pointCloud);
  addOutput(&d_pointCloudColors);
}

void FeatureTriangulator::update()
{
  std::cout << getName() << std::endl;

  common::matrix::sofaMat2cvMat(d_R.getValue(), R);
  common::matrix::sofaVector2cvMat(d_T.getValue(), T);
  common::matrix::sofaMat2cvMat(d_cmL.getValue(), cmL);
  common::matrix::sofaMat2cvMat(d_cmR.getValue(), cmR);
  common::matrix::sofaVector2cvMat(d_dvL.getValue(), dvL);
  common::matrix::sofaVector2cvMat(d_dvR.getValue(), dvR);
  PL = cv::Matx34d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
  PR = cv::Matx34d(R[0][0], R[0][1], R[0][2], T[0][0], R[1][0], R[1][1],
                   R[1][2], T[0][1], R[2][0], R[2][1], R[2][2], T[0][2]);

  helper::vector<Vec3d>& pts = *(d_pointCloud.beginWriteOnly());
  helper::vector<Vec3b>& colors = *(d_pointCloudColors.beginWriteOnly());

  const helper::vector<common::cvKeypoint>& kL = d_keypointsL.getValue();
  const helper::vector<common::cvKeypoint>& kR = d_keypointsR.getValue();
  const helper::SVector<helper::SVector<common::cvDMatch> >& m =
      d_matches.getValue();
  pts.resize(kL.size());
  colors.resize(kL.size());
  int sizePts = 0;
  (kL.size() > kR.size()) ? (sizePts = kR.size()) : (sizePts = kL.size());

  if (d_img.isSet() && !d_img.getValue().empty())
  {
    if (d_matches.isSet())
      for (size_t i = 0; i < m.size(); ++i)
      {
        cv::Point2f ptL = kL[m[i][0].queryIdx].pt;
        triangulate(ptL, kR[m[i][0].trainIdx].pt, pts[i]);
        cv::Vec3b c = d_img.getValue().at<cv::Vec3b>(ptL.y, ptL.x);
        colors[i] = Vec3b(c[0], c[1], c[2]);
      }
    else
    {
      for (size_t i = 0; i < sizePts; ++i)
      {
        triangulate(kL[i].pt, kR[i].pt, pts[i]);
        cv::Vec3b c = d_img.getValue().at<cv::Vec3b>(kL[i].pt.y, kL[i].pt.x);
        colors[i] = Vec3b(c[0], c[1], c[2]);
      }
    }
    d_pointCloud.endEdit();
    d_pointCloudColors.endEdit();
  }
  else
  {
    if (d_matches.isSet())
      for (size_t i = 0; i < m.size(); ++i)
        triangulate(kL[m[i][0].queryIdx].pt, kR[m[i][0].trainIdx].pt, pts[i]);
    else
      for (size_t i = 0; i < sizePts; ++i)
        triangulate(kL[i].pt, kR[i].pt, pts[i]);
    d_pointCloud.endEdit();
  }
}

cv::Point3d FeatureTriangulator::rectifyPoint(double x, double y,
                                              cv::Mat_<double> distortion_vec)
{
  double k1 = distortion_vec.at<double>(0);
  double k2 = distortion_vec.at<double>(1);
  double rr = x * x + y * y;

  double x_ = x * (1 + k1 * rr + k2 * rr * rr);
  double y_ = y * (1 + k1 * rr + k2 * rr * rr);

  return (cv::Point3d(x_, y_, 1.0));
}

cv::Mat_<double> FeatureTriangulator::iterativeLinearLSTriangulation(
    cv::Point3d u, cv::Point3d u1)
{
  double wi = 1, wi1 = 1;
  cv::Mat_<double> X(4, 1);

  cv::Mat_<double> X_ = linearLSTriangulation(u, u1);
  X(0) = X_(0);
  X(1) = X_(1);
  X(2) = X_(2);
  X(3) = 1.0;

  for (int i = 0; i < 10; i++)
  {  // Hartley suggests 10 iterations at most
    // recalculate weights
    double p2x = cv::Mat_<double>(cv::Mat_<double>(PL).row(2) * X)(0);
    double p2x1 = cv::Mat_<double>(cv::Mat_<double>(PR).row(2) * X)(0);

    // breaking point
    // if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

    wi = p2x;
    wi1 = p2x1;

    // reweight equations and solve
    cv::Matx43d A(
        (u.x * PL(2, 0) - PL(0, 0)) / wi, (u.x * PL(2, 1) - PL(0, 1)) / wi,
        (u.x * PL(2, 2) - PL(0, 2)) / wi, (u.y * PL(2, 0) - PL(1, 0)) / wi,
        (u.y * PL(2, 1) - PL(1, 1)) / wi, (u.y * PL(2, 2) - PL(1, 2)) / wi,
        (u1.x * PR(2, 0) - PR(0, 0)) / wi1, (u1.x * PR(2, 1) - PR(0, 1)) / wi1,
        (u1.x * PR(2, 2) - PR(0, 2)) / wi1, (u1.y * PR(2, 0) - PR(1, 0)) / wi1,
        (u1.y * PR(2, 1) - PR(1, 1)) / wi1, (u1.y * PR(2, 2) - PR(1, 2)) / wi1);

    cv::Mat_<double> B =
        (cv::Mat_<double>(4, 1) << -(u.x * PL(2, 3) - PL(0, 3)) / wi,
         -(u.y * PL(2, 3) - PL(1, 3)) / wi, -(u1.x * PR(2, 3) - PR(0, 3)) / wi1,
         -(u1.y * PR(2, 3) - PR(1, 3)) / wi1);

    cv::solve(A, B, X_, cv::DECOMP_SVD);
    X(0) = X_(0);
    X(1) = X_(1);
    X(2) = X_(2);
    X(3) = 1.0;
  }

  return X;
}

cv::Mat_<double> FeatureTriangulator::linearLSTriangulation(cv::Point3d u,
                                                            cv::Point3d u1)
{
  /**
  From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image
  understanding, 1997
  */
  cv::Matx43d A(u.x * PL(2, 0) - PL(0, 0), u.x * PL(2, 1) - PL(0, 1),
                u.x * PL(2, 2) - PL(0, 2), u.y * PL(2, 0) - PL(1, 0),
                u.y * PL(2, 1) - PL(1, 1), u.y * PL(2, 2) - PL(1, 2),
                u1.x * PR(2, 0) - PR(0, 0), u1.x * PR(2, 1) - PR(0, 1),
                u1.x * PR(2, 2) - PR(0, 2), u1.y * PR(2, 0) - PR(1, 0),
                u1.y * PR(2, 1) - PR(1, 1), u1.y * PR(2, 2) - PR(1, 2));
  cv::Matx41d B(-(u.x * PL(2, 3) - PL(0, 3)), -(u.y * PL(2, 3) - PL(1, 3)),
                -(u1.x * PR(2, 3) - PR(0, 3)), -(u1.y * PR(2, 3) - PR(1, 3)));

  cv::Mat_<double> X;
  cv::solve(A, B, X, cv::DECOMP_SVD);

  return X;
}

void FeatureTriangulator::triangulate(const cv::Point2f& l,
                                      const cv::Point2f& r,
                                      defaulttype::Vec3d& p)
{
  cv::Point3d u(l.x, l.y, 1.0);
  cv::Point3d u1(r.x, r.y, 1.0);

  // multiply the point by the inverse of the K matrix
  cv::Mat_<double> um = cmL.inv() * cv::Mat_<double>(u);
  cv::Mat_<double> um1 = cmR.inv() * cv::Mat_<double>(u1);

  if (d_rectify.getValue())
  {
    u.x = rectifyPoint(um(0), um(1), dvL).x;
    u.y = rectifyPoint(um(0), um(1), dvL).y;
    u1.x = rectifyPoint(um1(0), um1(1), dvR).x;
    u1.y = rectifyPoint(um1(0), um1(1), dvR).y;
  }
  else
  {
    u.x = um(0);
    u.y = um(1);
    u1.x = um1(0);
    u1.y = um1(1);
  }
  u.z = um(2);
  u1.z = um1(2);

  cv::Mat_<double> X = iterativeLinearLSTriangulation(u, u1);

  p = defaulttype::Vec3d(X(0), X(1), X(2));
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
