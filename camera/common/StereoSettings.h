#ifndef SOFACV_CAM_STEREOSETTINGS_H
#define SOFACV_CAM_STEREOSETTINGS_H

#include "CameraSettings.h"
#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
/**
 * @brief The StereoSettings class
 *
 * Stores the links to the Two CameraSettings used for Stereoscopy and the
 * camera's Essential and Fundamental matrix
 */
class SOFA_IMAGEPROCESSING_API StereoSettings : public ImplicitDataEngine
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
  SOFA_CLASS(StereoSettings, ImplicitDataEngine);

  /// StereoSettings ctor. All parameters are optional and can be set using
  /// calibration components or file loaders
  StereoSettings();

  ~StereoSettings() {}
  void init() override;
  virtual void doUpdate() override {}
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
  void FundamentalMatrixChanged();
  void EssentialMatrixChanged();

 private:
  cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u,
                                                  cv::Point3d u1);
  cv::Mat_<double> linearLSTriangulation(cv::Point3d u, cv::Point3d u1);
  void updateRt();
};

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_STEREOSETTINGS_H
