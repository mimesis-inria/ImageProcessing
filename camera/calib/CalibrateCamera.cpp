/******************************************************************************
 *       SOFAOR, SOFA plugin for the Operating Room, development version       *
 *                        (c) 2017 INRIA, MIMESIS Team                         *
 *                                                                             *
 * This program is a free software; you can redistribute it and/or modify it   *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 1.0 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: Bruno Marques and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact-mimesis@inria.fr                               *
 ******************************************************************************/

#include "CalibrateCamera.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
SOFA_DECL_CLASS(CalibrateCamera)

int CalibrateCameraClass =
    sofa::core::RegisterObject(
        "Component calibrating a monoscopic camera from a set of 2D and 3D "
        "correspondances. computed matrices are set in the linked "
        "CameraSettings, while the multiple translation vectors are rotation "
        "matrices estimated for each set of 2D/3D correspondences are made "
        "available for further processing")
        .add<CalibrateCamera>();

void CalibrateCamera::calibrate()
{
  std::vector<std::vector<cv::Point2f> > imgPts;
  std::vector<std::vector<cv::Point3f> > objPts;

  for (auto pts : d_objectPoints.getValue())
  {
    std::vector<cv::Point3f> objPoints;
    for (auto pt : pts)
      objPoints.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
    objPts.push_back(objPoints);
  }
  for (auto pts : d_imagePoints.getValue())
  {
    std::vector<cv::Point2f> imgPoints;
    for (auto pt : pts) imgPoints.push_back(cv::Point2f(pt.x(), pt.y()));
    imgPts.push_back(imgPoints);
  }

  //	cv::Mat_<double> camMatrix;
  cv::Mat camMatrix =
      (cv::Mat_<double>(3, 3) << 2000, 0, d_imgSize.getValue().x() / 2.0, 0,
       2000, d_imgSize.getValue().y() / 2.0, 0, 0, 1.0);
  cv::Mat dc;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  try
  {
    if (d_K.isSet()) common::matrix::sofaMat2cvMat(d_K.getValue(), camMatrix);
    if (d_distCoefs.isSet())
      common::matrix::sofaVector2cvMat(d_distCoefs.getValue(), dc);

    std::cout << cv::calibrateCamera(objPts, imgPts,
                                     cv::Size(d_imgSize.getValue().x(),
                                              d_imgSize.getValue().y()),
                                     camMatrix, dc, rvecs, tvecs,
                                     d_calibFlags.getValue())
              << std::endl;
  }
  catch (cv::Exception& e)
  {
    msg_error(getName() + "::calibrate()") << e.what();
  }

  common::matrix::cvMat2sofaVector(dc, m_distCoefs);

  common::matrix::cvMat2sofaMat(camMatrix, m_K);
  sofa::helper::vector<sofa::defaulttype::Mat3x4d>& RTs = *d_Rts.beginEdit();
  for (unsigned i = 0; i < rvecs.size(); ++i)
  {
    // get 3d rot mat
    cv::Mat rotM(3, 3, CV_64F);
    cv::Rodrigues(rvecs[i], rotM);

    // push tvec to transposed Mat
    // tvec is ALREADY the 3rd column of a 3x4 proj matrix... so just append it
    // to the Rotation matrix
    cv::Mat rotMT = rotM.t();
    rotMT.push_back(tvecs[0].reshape(1, 1));
    cv::Mat P = camMatrix * rotMT.t();

    sofa::defaulttype::Mat3x4d ProjMat;
    common::matrix::cvMat2sofaMat(P, ProjMat);

    RTs.push_back(ProjMat);
  }

  //	d_rvecs.endEdit();
  //	d_tvecs.endEdit();
  d_Rts.endEdit();
}

CalibrateCamera::CalibrateCamera()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters")),
      d_imagePoints(
          initData(&d_imagePoints, "imagePoints",
                   "a vector of vectors of the projections of the 3D "
                   "object's points in the image. imagePoints.size() and "
                   "objectPoints.size() and imagePoints[i].size() must "
                   "be equal to objectPoints[i].size() for each i")),
      d_objectPoints(initData(&d_objectPoints, "objectPoints",
                              "a vector of vectors of calibration pattern "
                              "points in the calibration pattern's coordinate "
                              "space")),
      d_imgSize(initData(&d_imgSize, "imageSize",
                         "size in px of the image (used to initialize the "
                         "intrinsic camera matrix")),
      d_calibFlags(
          initData(&d_calibFlags, 1, "calibFlags",
                   "One or a combination of the following flags:\n"
                   "USE_INTRINSIC_GUESS (1): cameraMatrix contains "
                   "valid initial values of fx, fy, cx, cy that are "
                   "optimized further.\n"
                   "FIX_ASPECT_RATIO (2): preserves the fx/fy ratio\n"
                   "FIX_PRINCIPAL_POINT (4): The principal point won't "
                   "change during optimization\n"
                   "ZERO_TANGENT_DIST (8): Tangential distortion is set to 0")),
      d_K(initData(&d_K, "K",
                   "Required for non-planar calbiration rigs, Optional "
                   "otherwise. used to provide initial guesses (depending on "
                   "used calibFlags)")),
      d_distCoefs(initData(&d_distCoefs, "distCoefs",
                           "[Optional] distortion coefficients initial guess "
                           "(check calibFlags)")),
      d_preserveExtrinsics(
          initData(&d_preserveExtrinsics, false, "keepExtrinsics",
                   "if true, only intrinsics are updated. Otherwise, "
                   "extrinsics are set to the last frame's pose estimation"))
{
}

void CalibrateCamera::init()
{
  addInput(&d_imagePoints);
  addInput(&d_objectPoints);
  addInput(&d_imgSize);
  addOutput(&d_K);
  addOutput(&d_distCoefs);

  if (!l_cam.get())
    msg_error(getName() + "::init()") << "Error: No camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";
  update();
}

void CalibrateCamera::Update()
{
  calibrate();

  const sofa::defaulttype::Matrix3& K = m_K;

  if (!d_preserveExtrinsics.getValue())
    l_cam->setProjectionMatrix(d_Rts.getValue().back());
  else
  {
    l_cam->setIntrinsicCameraMatrix(K, true);
  }
  l_cam->setDistortionCoefficients(m_distCoefs);
}

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
