#include <SofaORCommon/cvMatUtils.h>
#include "FeatureRectifier.h"

#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(FeatureRectifier)

int FeatureRectifierClass =
    core::RegisterObject(
        "component generating a 3D point cloud from two 2D keypoints list and "
        "their matches, and stereo camera parameters")
        .add<FeatureRectifier>();

FeatureRectifier::FeatureRectifier()
    : ImageFilter(),
      d_calib(initData(
          &d_calib, "calib",
          "camera calibration data (only the distorsion vector is used)")),
      d_keypoints_in(initData(&d_keypoints_in, "points_in",
                              "input vector of left keypoints", true, true)),
      d_keypoints_out(initData(&d_keypoints_out, "points_out",
                               "output keypoints, rectified"))
{
  m_outputImage = false;
}

FeatureRectifier::~FeatureRectifier() {}

void FeatureRectifier::init()
{
  getCalibFromContext();
  addInput(&d_calib);
  addInput(&d_keypoints_in);
  addOutput(&d_keypoints_out);

  setDirtyValue();
  ImageFilter::init();
}

void FeatureRectifier::update() { ImageFilter::update(); }

void FeatureRectifier::applyFilter(const cv::Mat& in, cv::Mat& out, bool debug)
{
  common::matrix::sofaVector2cvMat(d_calib.getValue().distCoefs, dist);

  helper::vector<common::cvKeypoint>& kptsOut = *d_keypoints_out.beginWriteOnly();
  const helper::vector<common::cvKeypoint>& kptsIn = d_keypoints_in.getValue();
  kptsOut.reserve(kptsIn.size());

  for (common::cvKeypoint kp : kptsIn)
  {
      double x = kp.pt.x;
      double y = kp.pt.y;
      double k1 = dist.at<double>(0);
      double k2 = dist.at<double>(1);
      double rr = x * x + y * y;

      kp.pt.x = x * (1 + k1 * rr + k2 * rr * rr);
      kp.pt.y = y* (1 + k1 * rr + k2 * rr * rr);

      kptsOut.push_back(kp);
  }
}

void FeatureRectifier::reinit() { ImageFilter::reinit(); }
}  // namespace processor
}  // namespace OR
}  // namespace sofa
