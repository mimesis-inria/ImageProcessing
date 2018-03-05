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

#include "CalibrateStereo.h"
#include <SofaORCommon/cvMatUtils.h>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
SOFA_DECL_CLASS(CalibrateStereo)

int CalibrateStereoClass =
    sofa::core::RegisterObject(
        "Component calibrating a Stereoscopic camera from two sets of 2D and "
        "3D correspondances respectively referring to the reference camera and "
        "the secondary camera. Computed Extrinsics are set in the linked "
        "StereoSettings")
        .add<CalibrateStereo>();

void CalibrateStereo::calibrate()
{
  std::vector<std::vector<cv::Point3f> > objectPoints;
  std::vector<std::vector<cv::Point2f> > imagePoints1;
  std::vector<std::vector<cv::Point2f> > imagePoints2;

  if (d_imagePoints1.getValue().size() != d_imagePoints2.getValue().size() ||
      d_imagePoints1.getValue().size() < 1 ||
      d_imagePoints1.getValue()[0].size() !=
          d_imagePoints2.getValue()[0].size() ||
      d_imagePoints1.getValue()[0].size() !=
          d_objectPoints.getValue()[0].size())
    msg_error(getName() + "::calibrate()") << "Error: Vector size should be "
                                              "the same for imagePoints1, "
                                              "imagePoint2 and objectPoints";

  for (auto pts : d_objectPoints.getValue())
  {
    std::vector<cv::Point3f> objPts;
    for (auto pt : pts)
    {
      objPts.push_back(cv::Point3f(pt.x(), pt.y(), pt.z()));
    }
    objectPoints.push_back(objPts);
  }
  for (auto pts : d_imagePoints1.getValue())
  {
    std::vector<cv::Point2f> imgPts;
    for (auto pt : pts)
    {
      imgPts.push_back(cv::Point2f(pt.x(), pt.y()));
    }
    imagePoints1.push_back(imgPts);
  }
  for (auto pts : d_imagePoints2.getValue())
  {
    std::vector<cv::Point2f> imgPts;
    for (auto pt : pts)
    {
      imgPts.push_back(cv::Point2f(pt.x(), pt.y()));
    }
    imagePoints2.push_back(imgPts);
  }

  cv::Mat_<double> distCoeffs1;
  cv::Mat_<double> distCoeffs2;
  cv::Mat Rmat;
  cv::Mat Tvec;
  cv::Mat E;
  cv::Mat F;

  cv::Mat_<double> cam1, cam2;
  common::matrix::sofaMat2cvMat(l_cam->getCamera1().getIntrinsicCameraMatrix(),
                                cam1);
  common::matrix::sofaMat2cvMat(l_cam->getCamera2().getIntrinsicCameraMatrix(),
                                cam2);

  common::matrix::sofaVector2cvMat(
      l_cam->getCamera1().getDistortionCoefficients(), distCoeffs1);
  common::matrix::sofaVector2cvMat(
      l_cam->getCamera2().getDistortionCoefficients(), distCoeffs2);

  std::cout << "reprojectionError: "
            << cv::stereoCalibrate(
                   objectPoints, imagePoints1, imagePoints2, cam1, distCoeffs1,
                   cam2, distCoeffs2,
                   cv::Size(d_imgSize.getValue().x(), d_imgSize.getValue().y()),
                   Rmat, Tvec, E, F,
                   cv::CALIB_FIX_INTRINSIC | cv::CALIB_USE_INTRINSIC_GUESS)
            << std::endl;

  //	l_cam->setRotationMatrix(defaulttype::Matrix3((double*)Rmat.ptr()));
  //	l_cam->setTranslationVector(defaulttype::Vector3((double*)Tvec.ptr()));
  l_cam->setEssentialMatrix(sofa::defaulttype::Matrix3((double*)E.ptr()));
  l_cam->setFundamentalMatrix(sofa::defaulttype::Matrix3((double*)F.ptr()));
}

CalibrateStereo::CalibrateStereo()
    : l_cam(initLink("cam",
                     "link to the CameraSettings component containing and "
                     "maintaining the reference camera's parameters")),
      d_imagePoints1(
          initData(&d_imagePoints1, "imagePoints1",
                   "a vector of vectors of the projections of the 3D "
                   "object's points in the reference camera's image. "
                   "imagePoints1.size() and "
                   "objectPoints.size() and imagePoints1[i].size() must "
                   "be equal to objectPoints1[i].size() for each i")),
      d_imagePoints2(
          initData(&d_imagePoints2, "imagePoints2",
                   "a vector of vectors of the projections of the 3D "
                   "object's points in the second camera's image. "
                   "imagePoints2.size() and "
                   "objectPoints.size() and imagePoints2[i].size() must "
                   "be equal to objectPoints[i].size() for each i")),
      d_objectPoints(initData(&d_objectPoints, "objectPoints",
                              "a vector of vectors of calibration pattern "
                              "points in the calibration pattern's coordinate "
                              "space")),
      d_imgSize(initData(&d_imgSize, "imageSize",
                         "size in px of the image (used to initialize the "
                         "intrinsic camera matrix")),
      d_calibFlags(
          initData(&d_calibFlags, 0x00101, "calibFlags",
                   "One or a combination of the following flags:\n"
                   "USE_INTRINSIC_GUESS (1): cameraMatrix contains "
                   "valid initial values of fx, fy, cx, cy that are "
                   "optimized further.\n"
                   "FIX_ASPECT_RATIO (2): preserves the fx/fy ratio\n"
                   "FIX_PRINCIPAL_POINT (4): The principal point won't "
                   "change during optimization\n"
                   "ZERO_TANGENT_DIST (8): Tangential distortion is set to 0"))
{
}

void CalibrateStereo::init()
{
  addInput(&d_imagePoints1);
  addInput(&d_imagePoints2);
  addInput(&d_objectPoints);
  addInput(&d_imgSize);

  if (!l_cam.get())
    msg_error(getName() + "::init()")
        << "Error: No Stereo camera settings link set. "
           "Please use attribute 'cam' "
           "to define one";
  update();
}

void CalibrateStereo::Update()
{
  if (d_imagePoints1.isSet() && d_imagePoints2.isSet() &&
      d_objectPoints.isSet())
    calibrate();
  else
  {
    std::cout << "computing stereo params from cameras" << std::endl;
    l_cam->recomputeFromCameras();
  }
}

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
