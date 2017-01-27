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
    : ImageFilter(),
      d_rectify(initData(
          &d_rectify, false, "rectify",
          "if set to true, points will be rectified before triangulating")),
      d_extrinsics(initData(&d_extrinsics, "extrinsics",
                            "extrinsic parameters of a stereo camera", true,
                            true)),
      d_camLeft(initData(&d_camLeft, "cam_left",
                         "left intrinsics camera params", true, true)),
      d_camRight(initData(&d_camRight, "cam_right",
                          "right intrinsics camera params", true, true)),
      d_keypointsL(initData(&d_keypointsL, "kpts_l",
                            "input vector of left keypoints", true, true)),
      d_keypointsR(initData(&d_keypointsR, "kpts_l",
                            "input vector of right keypoints", true, true)),
      d_matches(initData(&d_matches, "matches",
                         "input array of matches between the 2 vectors "
                         "(optional if keypoints are already sorted).",
                         true, true)),
      d_pointCloud(initData(&d_pointCloud, "pc", "output vector of 3D points"))

{
  m_outputImage = false;
}

FeatureTriangulator::~FeatureTriangulator() {}
void FeatureTriangulator::init()
{
  addInput(&d_extrinsics);
  addInput(&d_camLeft);
  addInput(&d_camRight);
  addInput(&d_matches);
  addInput(&d_keypointsL);
  addInput(&d_keypointsR);
  addOutput(&d_pointCloud);

  setDirtyValue();
  ImageFilter::init();
}

void FeatureTriangulator::update() { ImageFilter::update(); }
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
    cv::Point3d u, cv::Matx34d P, cv::Point3d u1, cv::Matx34d P1)
{
  double wi = 1, wi1 = 1;
  cv::Mat_<double> X(4, 1);

  cv::Mat_<double> X_ = linearLSTriangulation(u, P, u1, P1);
  X(0) = X_(0);
  X(1) = X_(1);
  X(2) = X_(2);
  X(3) = 1.0;

  for (int i = 0; i < 10; i++)
  {  // Hartley suggests 10 iterations at most
    // recalculate weights
    double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2) * X)(0);
    double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2) * X)(0);

    // breaking point
    // if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

    wi = p2x;
    wi1 = p2x1;

    // reweight equations and solve
    cv::Matx43d A(
        (u.x * P(2, 0) - P(0, 0)) / wi, (u.x * P(2, 1) - P(0, 1)) / wi,
        (u.x * P(2, 2) - P(0, 2)) / wi, (u.y * P(2, 0) - P(1, 0)) / wi,
        (u.y * P(2, 1) - P(1, 1)) / wi, (u.y * P(2, 2) - P(1, 2)) / wi,
        (u1.x * P1(2, 0) - P1(0, 0)) / wi1, (u1.x * P1(2, 1) - P1(0, 1)) / wi1,
        (u1.x * P1(2, 2) - P1(0, 2)) / wi1, (u1.y * P1(2, 0) - P1(1, 0)) / wi1,
        (u1.y * P1(2, 1) - P1(1, 1)) / wi1, (u1.y * P1(2, 2) - P1(1, 2)) / wi1);

    cv::Mat_<double> B =
        (cv::Mat_<double>(4, 1) << -(u.x * P(2, 3) - P(0, 3)) / wi,
         -(u.y * P(2, 3) - P(1, 3)) / wi, -(u1.x * P1(2, 3) - P1(0, 3)) / wi1,
         -(u1.y * P1(2, 3) - P1(1, 3)) / wi1);

    cv::solve(A, B, X_, cv::DECOMP_SVD);
    X(0) = X_(0);
    X(1) = X_(1);
    X(2) = X_(2);
    X(3) = 1.0;
  }

  return X;
}

cv::Mat_<double> FeatureTriangulator::linearLSTriangulation(cv::Point3d u,
                                                            cv::Matx34d P,
                                                            cv::Point3d u1,
                                                            cv::Matx34d P1)
{
  /**
  From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image
  understanding, 1997
  */
  cv::Matx43d A(u.x * P(2, 0) - P(0, 0), u.x * P(2, 1) - P(0, 1),
                u.x * P(2, 2) - P(0, 2), u.y * P(2, 0) - P(1, 0),
                u.y * P(2, 1) - P(1, 1), u.y * P(2, 2) - P(1, 2),
                u1.x * P1(2, 0) - P1(0, 0), u1.x * P1(2, 1) - P1(0, 1),
                u1.x * P1(2, 2) - P1(0, 2), u1.y * P1(2, 0) - P1(1, 0),
                u1.y * P1(2, 1) - P1(1, 1), u1.y * P1(2, 2) - P1(1, 2));
  cv::Matx41d B(-(u.x * P(2, 3) - P(0, 3)), -(u.y * P(2, 3) - P(1, 3)),
                -(u1.x * P1(2, 3) - P1(0, 3)), -(u1.y * P1(2, 3) - P1(1, 3)));

  cv::Mat_<double> X;
  cv::solve(A, B, X, cv::DECOMP_SVD);

  return X;
}

void FeatureTriangulator::triangulate(const cv::Point2f& l,
                                      const cv::Point2f& r,
                                      defaulttype::Vec3d& p)
{
  cv::Matx34d PL = cv::Matx34d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);

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

  cv::Matx34d PR =
      cv::Matx34d(R[0][0], R[0][1], R[0][2], T[0][0], R[1][0], R[1][1], R[1][2],
                  T[0][1], R[2][0], R[2][1], R[2][2], T[0][2]);

  cv::Mat_<double> X = iterativeLinearLSTriangulation(u, PL, u1, PR);

  p = defaulttype::Vec3d(X(0), X(1), X(2));
}

void FeatureTriangulator::applyFilter(const cv::Mat&, cv::Mat&, bool)
{
  if (!f_listening.getValue()) return;

  common::matrix::sofaMat2cvMat(d_extrinsics.getValue().R, R);
  common::matrix::sofaVector2cvMat(d_extrinsics.getValue().T, T);
  common::matrix::sofaMat2cvMat(d_camLeft.getValue().cameraMatrix, cmL);
  common::matrix::sofaMat2cvMat(d_camRight.getValue().cameraMatrix, cmR);
  common::matrix::sofaVector2cvMat(d_camLeft.getValue().distCoefs, dvL);
  common::matrix::sofaVector2cvMat(d_camRight.getValue().distCoefs, dvR);

  helper::vector<defaulttype::Vec3d>& pts = *(d_pointCloud.beginWriteOnly());
  const helper::vector<common::cvKeypoint>& kL = d_keypointsL.getValue();
  const helper::vector<common::cvKeypoint>& kR = d_keypointsR.getValue();
  const helper::SVector<helper::SVector<common::cvDMatch> >& m =
      d_matches.getValue();
  pts.resize(kL.size());

  if (m.size() == pts.size())
    for (size_t i = 0; i < m.size(); ++i)
      triangulate(kL[m[i][0].queryIdx].pt, kR[m[i][0].trainIdx].pt, pts[i]);
  else
    for (size_t i = 0; i < pts.size(); ++i)
      triangulate(kL[i].pt, kR[i].pt, pts[i]);
}

void FeatureTriangulator::reinit() { ImageFilter::reinit(); }
}  // namespace processor
}  // namespace OR
}  // namespace sofa
