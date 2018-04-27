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

#include "SolvePnP.h"
#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace cam
{
namespace calib
{
SOFA_DECL_CLASS(SolvePnP)

int SolvePnPClass =
    sofa::core::RegisterObject(
        "The Component estimates the object pose given a set of object points, "
        "their corresponding image projections, as well as the camera matrix "
        "and the distortion coefficients.")
        .add<SolvePnP>();

SolvePnP::SolvePnP()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters")),
      d_imagePoints(
          initData(&d_imagePoints, "imagePoints",
                   "a vector of 2D projections of the 3D "
                   "object's points in the image. imagePoints.size() and "
                   "objectPoints.size() must "
                   "be equal")),
      d_objectPoints(initData(&d_objectPoints, "objectPoints",
                              "a vector of calibration pattern "
                              "points in the calibration pattern's coordinate "
                              "space")),
      d_imgSize(initData(&d_imgSize, "imageSize",
                         "[Optional] size in px of the image. If not "
                         "provided, recovered from CameraSettings")),
      d_K(initData(&d_K, "K",
                   "[Optional] Intrinsic matrix. If not provided, intrinsics "
                   "will be recovered from CameraSettings, or approximated "
                   "from the imageSize")),
      d_distCoefs(initData(&d_distCoefs, "distCoefs",
                           "[Optional] distortion coefficients initial guess")),
      d_pnpFlags(initData(
          &d_pnpFlags, 4, "pnpFlags",
          "One or a combination of the following flags:\n"
          "ITERATIVE (1): Iterative method based on Levenberg-Marquardt "
          "optimization.\n"
          "P3P (2): Method based on the paper of X.S. Gao, X.-R. Hou, J. "
          "Tang, H.-F. Chang “Complete Solution Classification for the "
          "Perspective-Three-Point Problem”. In this case the function "
          "requires exactly four object and image points.\n"
          "EPNP (4): Method introduced by F.Moreno-Noguer, V.Lepetit and "
          "P.Fua in the paper “EPnP: Efficient Perspective-n-Point Camera "
          "Pose Estimation\n"))
{
}

void SolvePnP::init()
{
  addInput(&d_imagePoints);
  addInput(&d_objectPoints);
  addInput(&d_imgSize);
  addInput(&d_K);
  addInput(&d_distCoefs);

  if (!(l_cam.get()))
    msg_error(this->getName() + "::init()") << "Error: No camera link set. "
                                               "Please use attribute 'cam' "
                                               "to define one";
  update();
}

void SolvePnP::Update()
{
  std::vector<cv::Point2d> imgPts;
  std::vector<cv::Point3d> objPts;

  for (auto pt : d_objectPoints.getValue())
    objPts.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
  for (auto pt : d_imagePoints.getValue())
    imgPts.push_back(cv::Point2f(pt.x(), pt.y()));

  cv::Mat_<double> camMatrix;
  cv::Mat_<double> dc;
  cv::Mat rvec, tvec;
  sofa::defaulttype::Vec2i imsize = d_imgSize.getValue();
  try
  {
    if (d_K.isSet())
      matrix::sofaMat2cvMat(d_K.getValue(), camMatrix);
    else if (l_cam->getIntrinsicCameraMatrix() ==
             sofa::defaulttype::Matrix3::Identity())
      matrix::sofaMat2cvMat(l_cam->getIntrinsicCameraMatrix(),
                                    camMatrix);
    else
    {
      int max_d;
      if (d_imgSize.isSet())
        imsize = d_imgSize.getValue();
      else
        imsize = l_cam->getImageSize();
      max_d = std::max(imsize.x(), imsize.y());
      camMatrix = (cv::Mat_<double>(3, 3) << max_d, 0, imsize.x() / 2.0, 0,
                   max_d, imsize.y() / 2.0, 0, 0, 1.0);
    }
    if (d_distCoefs.isSet())
      matrix::sofaVector2cvMat(d_distCoefs.getValue(), dc);
    else
      matrix::sofaVector2cvMat(l_cam->getDistortionCoefficients(), dc);

    cv::solvePnP(objPts, imgPts, camMatrix, dc, rvec, tvec, false,
                 d_pnpFlags.getValue());
  }
  catch (cv::Exception& e)
  {
    msg_error(getName() + "::update()") << e.what();
    return;
  }

  sofa::defaulttype::Mat3x4d P;
  cv::Mat rotM(3, 3, CV_64F);
  cv::Rodrigues(rvec, rotM);

  // push tvec to transposed Mat
  cv::Mat rotMT = rotM.t();
  rotMT.push_back(tvec.reshape(1, 1));

  // transpose back, and multiply
  cv::Mat Proj = camMatrix * rotMT.t();

  for (unsigned j = 0; j < 3; j++)
  {
    for (unsigned i = 0; i < 4; i++)
    {
      P[j][i] = Proj.at<double>(j, i);
    }
  }

  sofa::helper::vector<double> distCoefs;
  matrix::cvMat2sofaVector(dc, distCoefs);

  if (l_cam->getImageSize() != imsize)
    l_cam->setImageSize(d_imgSize.getValue());
  l_cam->setProjectionMatrix(P);
  l_cam->setDistortionCoefficients(distCoefs);
}

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
