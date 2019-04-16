#include "FeatureTriangulator.h"
#include <SofaCV/SofaCV.h>

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace cam
{
SOFA_DECL_CLASS(FeatureTriangulator)

int FeatureTriangulatorClass =
    sofa::core::RegisterObject(
        "component generating a 3D point cloud from two 2D keypoints list and "
        "their matches, and stereo camera parameters")
        .add<FeatureTriangulator>();

FeatureTriangulator::FeatureTriangulator()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters")),
      d_keypointsL(initData(&d_keypointsL, "keypoints1",
                            "input vector of left keypoints", true, true)),
      d_keypointsR(initData(&d_keypointsR, "keypoints2",
                            "input vector of right keypoints", true, true)),
      d_matches(initData(
          &d_matches, "matches",
          "input array of matches (optional if keypoints are already sorted).",
          true, true)),
      d_pointCloud(
          initData(&d_pointCloud, "positions", "output vector of 3D points"))
{
  f_listening.setValue(true);
  addAlias(&d_pointCloud, "positions_out");
}

FeatureTriangulator::~FeatureTriangulator() {}
void FeatureTriangulator::init()
{
  addInput(&d_matches);
  addInput(&d_keypointsL);
  addInput(&d_keypointsR);

  addOutput(&d_pointCloud);

  if (!l_cam.get())
    msg_error(getName() + "::init()") << "Error: No stereo camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";

  update();
}

void FeatureTriangulator::doUpdate()
{
  //	common::matrix::sofaMat2cvMat(l_cam->getRotationMatrix(), R);
  //	common::matrix::sofaVector2cvMat(l_cam->getTranslationVector(), T);
  matrix::sofaMat2cvMat(l_cam->getCamera1().getProjectionMatrix(), cmL);
  matrix::sofaMat2cvMat(l_cam->getCamera2().getProjectionMatrix(), cmR);
  matrix::sofaVector2cvMat(l_cam->getCamera1().getDistortionCoefficients(),
                           dvL);
  matrix::sofaVector2cvMat(l_cam->getCamera2().getDistortionCoefficients(),
                           dvR);

  PL = cmL;
  PR = cmR;

  sofa::helper::vector<Vec3d>& pts = *(d_pointCloud.beginWriteOnly());

  const sofa::helper::vector<cvKeypoint>& kL = d_keypointsL.getValue();
  const sofa::helper::vector<cvKeypoint>& kR = d_keypointsR.getValue();
  pts.resize(kL.size());
  size_t sizePts = 0;
  (kL.size() > kR.size()) ? (sizePts = kR.size()) : (sizePts = kL.size());

  if (d_matches.isSet())
  {
    const sofa::helper::vector<cvDMatch>& m = d_matches.getValue();
    for (size_t i = 0; i < m.size(); ++i)
      pts[i] = l_cam->triangulate(kL[m[i].queryIdx].pt, kR[m[i].trainIdx].pt);
  }
  else
    for (size_t i = 0; i < sizePts; ++i)
      pts[i] = l_cam->triangulate(kL[i].pt, kR[i].pt);
  d_pointCloud.endEdit();
}

}  // namespace cam
}  // namespace sofacv
