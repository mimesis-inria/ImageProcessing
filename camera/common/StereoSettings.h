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

#ifndef SOFA_OR_PROCESSOR_STEREOSETTINGS_H
#define SOFA_OR_PROCESSOR_STEREOSETTINGS_H

#include "CameraSettings.h"
#include "ProcessOR/initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
/**
 * @brief The StereoSettings class
 *
 * Stores the links to the Two CameraSettings used for Stereoscopy and the
 * camera's Essential and Fundamental matrix
 */
class StereoSettings : public common::ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      StereoSettings, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  typedef sofa::defaulttype::Vector3 Vector3;
  typedef sofa::defaulttype::Vector2 Vector2;
  typedef sofa::defaulttype::Matrix3 Matrix3;

 public:
  SOFA_CLASS(StereoSettings, common::ImplicitDataEngine);

  /// StereoSettings ctor. All parameters are optional and can be set using
  /// calibration components or file loaders
  StereoSettings();

  ~StereoSettings() {}
  void init();
  void update() {}
  /// returns the 3D position of a pair of 2D matches 'X, Y'
  Vector3 triangulate(const Vector2& x1, const Vector2& x2);
  /// returns the 3D position of a pair of 2D matches 'X, Y'
  Vector3 triangulate(const cv::Point2d& x1, const cv::Point2d& x2);
  /// returns the 3D position of a pair of 2D matches 'X, Y'
  void triangulate(const Vector2& x1, const Vector2& x2, Vector3& w);
  /// returns the 3D position of a pair of 2D matches 'X, Y'
  void triangulate(const cv::Point2d& x1, const cv::Point2d& x2, Vector3& w);

  /// Returns the Fundamental Matrix F
  const Matrix3& getFundamentalMatrix();
  /// sets the Fundamental Matrix F
  void setFundamentalMatrix(const Matrix3& F);

  /// Returns the Essential Matrix E
  const Matrix3& getEssentialMatrix();
  /// Sets the Essential Matrix E
  void setEssentialMatrix(const Matrix3& E);

  /// returns the reference camera's settings
  CameraSettings& getCamera1();
  /// returns the second camera's settings
  CameraSettings& getCamera2();

  /// Recomputes F and E from the two CameraSettings
  void recomputeFromCameras();

 private:
  CamSettings l_cam1;       ///< Reference Cam
  CamSettings l_cam2;       ///< Second cam
  sofa::Data<Matrix3> d_F;  ///< Fundamental Mat
  sofa::Data<Matrix3> d_E;  ///< Essential Mat

  //	cv::Mat_<double> K1, K2;
  cv::Mat_<double> P1, P2;

 public:
  // Data callbacks for GUI
  void FundamentalMatrixChanged(sofa::core::objectmodel::BaseData*);
  void EssentialMatrixChanged(sofa::core::objectmodel::BaseData*);

 private:
  cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u,
                                                  cv::Point3d u1);
  cv::Mat_<double> linearLSTriangulation(cv::Point3d u, cv::Point3d u1);
  void updateRt();
};

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_STEREOSETTINGS_H
