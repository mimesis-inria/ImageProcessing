#ifndef SOFACV_CAM_CAMERASETTINGS_H
#define SOFACV_CAM_CAMERASETTINGS_H

#include <ImageProcessing/ImageProcessingPlugin.h>

#include <SofaCV/SofaCV.h>

#include <sofa/gl/DrawToolGL.h>
#include <sofa/core/visual/VisualManager.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/Transformation.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/gl/template.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glu.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
/**
 * \brief Camera-related stuff
 */
namespace cam
{
/**
 * @brief The CameraSettings class
 *
 * This component holds the monoscopic camera parameters and maintain their
 * 4 different representations up-to-date at all time:
 * - OpenGL representation (4x4 modelview + projection matrix)
 * - "Vision" representation: (3x4 projection matrix M)
 * - "OpenCV" representation: (3x3 K, R, and vector t)
 * - "Decomposed" representation: 2D scale, skew and translation matrix,
 *    camera position, orientation etc.
 */
class SOFA_IMAGEPROCESSING_API CameraSettings : public ImplicitDataEngine
{
 public:
  typedef sofa::defaulttype::RigidTypes::Coord Rigid;
  typedef sofa::type::Vector2 Vector2;
  typedef sofa::type::Vector3 Vector3;
  typedef sofa::type::Vec<4, int> Vector4;
  typedef sofa::type::Vec<2, int> Vec2i;
  typedef sofa::type::Vec<5, double> Vector5;
  typedef sofa::type::Mat3x4d Mat3x4d;
  typedef sofa::type::Matrix4 Matrix4;
  typedef sofa::type::Matrix3 Matrix3;
  typedef sofa::type::Quaternion Quat;

 public:
  SOFA_CLASS(CameraSettings, ImplicitDataEngine);

  /**
   * There are multiple ways of initializing this component, but the imageSize
   * and focal distance should always be provided. Their default value are
   * 1280x720 and 1.0 and can be left that way if satisfying.
   * - By simply creating the component with its default parameters,
   *   CameraSettings will initialize itself according to OpenGL's current
   *   modelview and projection matrix.
   * - One can also set the right parameters a-posteriori using another
   *   component such as SolvePnP or CalibrateCamera
   * - The camera settings can be defined using only 5 points: the position of
   *   the camera in world coordinates (C), and the position of the 4 corners of
   *   the image plane in world coordinates
   *   (corners3D)
   * - otherwise the camera settings will be extracted from either:
   * - M if provided
   * - K, R and t if provided
   * - glModelview, glProjection if provided
   * - The Opengl modelview / projection matrix currently set in OpenGL's
   *   context
   */
  CameraSettings();

  ~CameraSettings() {}

  /// computes all camera settings from K, t, lookAt and upVector
  void buildFromIntrinsicCamPosLookAtAndUpVector();
  /// computes all camera settings from K, t, upVector and fwdVector
  void buildFromIntrinsicCamPosUpVectorAndFwdVector();
  /// computes all camera settings from t and 3DCorners
  void buildFromCamPosAndImageCorners();
  /// computes all camera settings from M
  void buildFromM();
  /// computes all camera settings from K, R and t
  void buildFromKRT();
  /// computes all camera settings from glModelView and glProjection
  void buildFromOpenGL();
  /// computes all camera settings from opengl's current context
  void buildFromOpenGLContext();

  void init() override;
  void doUpdate() override;
  void reinit() override;

  /// returns the 2D pixel position of a given 3D point
  Vector2 get2DFrom3DPosition(const Vector3& p);

  /// returns the 3D position of a 2D point 'x, y'
  Vector3 get3DFrom2DPosition(double x, double y, double fz = -1);
  /// returns the 3D position of a 2D point p
  Vector3 get3DFrom2DPosition(const Vector2& p, double fz = -1);

  /// Returns the 3D corners of the image plane
  void getCornersPosition(Vector3& p1, Vector3& p2, Vector3& p3, Vector3& p4,
                          double fz = -1);

  /// returns the global projection matrix M
  const Mat3x4d& getProjectionMatrix() const;
  /// sets the global projection matrix M
  void setProjectionMatrix(const Mat3x4d& M);

  /// returns the intrinsic matrix K
  const Matrix3& getIntrinsicCameraMatrix() const;
  /// sets the intrinsic matrix K
  void setIntrinsicCameraMatrix(const Matrix3& K, bool update = true);

  /// returns the distortion coefficients
  const sofa::type::vector<double>& getDistortionCoefficients() const;
  /// sets the distortion coefficients
  void setDistortionCoefficients(const sofa::type::vector<double>& distCoefs);

  /// returns the rotation matrix R expressed in the world frame
  const Matrix3& getRotationMatrix() const;
  /// sets the rotation matrix R expressed in the world frame
  void setRotationMatrix(const Matrix3& R, bool update = true);

  /// returns the position of the camera's optical center in world coordinates
  const Vector3& getPosition() const;
  /// sets the position of the camera's optical center in world coordinates
  void setPosition(const Vector3& t, bool update = true);

  /// returns the image size in pixels
  const Vec2i& getImageSize() const;
  /// sets the image size in pixels
  void setImageSize(const Vec2i& imgSize, bool update = true);

  /// returns OpenGL's 4x4 projection matrix
  const Matrix4& getGLProjection() const;
  /// sets OpenGL's 4x4 projection matrix
  void setGLProjection(const Matrix4& glProjection);

  /// returns OpenGL's 4x4 modelview matrix
  const Matrix4& getGLModelview() const;
  /// sets OpenGL's 4x4 modelview matrix
  void setGLModelview(const Matrix4& glModelview);

  /// returns OpenGL's viewport
  const Vector4& getGLViewport() const;
  /// sets OpenGL's viewport
  void setGLViewport(const Vector4& glViewport);

  /// returns OpenGL's z clipping distances
  const Vector2& getGLZClip() const;
  /// sets OpenGL's z clipping distances
  void setGLZClip(const Vector2& zClip);

  /// returns the camera's orientation in OpenGL coordinates (Z flipped)
  const Quat& getOrientation() const;
  /// sets the camera's orientation in OpenGL coordinates (Z flipped)
  void setOrientation(const Quat& camPos);

  /// returns the Intrinsic's scaling matrix (focal opening)
  const Matrix3& get2DScaleMatrix() const;
  /// sets the Intrinsic's scaling matrix (focal opening)
  void set2DScaleMatrix(const Matrix3& scale2DMat);

  /// returns the focal distance (distance optical center / image plane)
  double getFocalDistance() const;
  /// sets the focal distance (distance optical center / image plane)
  void setFocalDistance(double f);

  /// returns the Intrinsic's translation matrix (Principal point position)
  const Matrix3& get2DTranslationMatrix() const;
  /// sets the Intrinsic's translation matrix (Principal point position)
  void set2DTranslationMatrix(const Matrix3& translation2DMat);

  /// returns the corners of the image plane at the current focale distance
  const sofa::type::vector<sofa::type::Vector3>& getCorners() const;

  /// returns the axis skew parameter (Always 0 in this implementation)
  double getAxisSkew() const;
  /// sets the axis skew parameter (Always 0 in this implementation)
  void setAxisSkew(double s);

  /// returns true if the camera is an XRay device
  bool isXRay() const;
  /// sets whether the camera is an XRay device
  void setXRay(bool isXray);

 public:
  sofa::Data<Vec2i> d_imageSize;      ///< Dimensions of the image in pixels
  sofa::Data<double> d_f;             ///< Focal distance
  sofa::Data<Matrix3> d_translate2D;  ///< 2D translation matrix (u0 v0)
  sofa::Data<Matrix3> d_scale2D;  ///< 2D scale matrix (focal opening, fx, fy)
  sofa::Data<sofa::type::vector<double> >
      d_distCoefs;          ///< camera's distortion coefficients if any
  sofa::Data<Mat3x4d> d_M;  ///< 3x4 global projection matrix
  sofa::Data<Matrix3> d_K;  ///< 3x3 Intrinsic Matrix
  sofa::Data<Matrix3> d_R;  ///< 3x3 rotation matrix

  sofa::Data<Vector3>
      d_t;  ///< Position in world coordinates of the camera's optical center

  sofa::Data<Matrix4> d_glProjection;  ///< 4x4 Opengl Projection matrix
  sofa::Data<Matrix4> d_glModelview;   ///< 4x4 Opengl Modelview matrix
  sofa::Data<Quat> d_orientation;      ///< Quaternion representation of the
  /// Rotation Matrix, with the Z axis flipped
  sofa::Data<Vector4> d_glViewport;  ///< OpenGL's viewport
  sofa::Data<Vector2> d_zClip;       ///< zNear, zFar

  sofa::Data<sofa::type::vector<sofa::type::Vector3> >
      d_3DCorners;  ///< [OUTPUT] 3D positions of the Image plane's corners
  sofa::Data<Vector3> d_upVector;  ///< camera's up vector

  sofa::Data<Vector3>
      d_fwdVector;  ///< direction cam -> lookat (where lookat is
                    /// often the 3D position of the projected
                    /// Principal Point)
  sofa::Data<Vector3>
      d_lookAt;  ///< target position (a point on the direction camPos
                 /// -> fwdVector)
  /// in CalibratedCamera, it is necessary to reverse depth when rasterizing
  /// models, to ensure that the view comes from behind the detector towards the
  /// source, opposite to the standard pinhole camera model
  sofa::Data<bool> d_isXRay;  ///< Whether or not this camera is an XRay
                              /// source-detector model (in which case,

  int index0;
  std::ofstream file;

 private:
  /// Decomposes the global projection matrix
  void decomposeM();
  /// Decomposes OpenGL modelview and projection matrix
  void decomposeGL();
  /// decomposes OpenCV's K, R and t
  void decomposeCV();
  /// Composes OpenGL's Modelview and Projection matrices
  void composeGL();
  /// Composes K, R, t and camPos
  void composeCV();
  /// Composes the global projection matrix
  void composeM();
  /// decomposes the Intrinsic matrix
  void decomposeK(const Matrix3& K);

 public:
  void ProjectionMatrixChanged();
  void IntrinsicCameraMatrixChanged();
  void DistortionCoefficientsChanged();
  void RotationMatrixChanged();
  void TranslationVectorChanged();
  void ImageSizeChanged();
  void GLProjectionChanged();
  void GLModelviewChanged();
  void GLViewportChanged();
  void GLZClipChanged();
  void OrientationChanged();
  void Scale2DChanged();
  void FocalDistanceChanged();
  void Translation2DChanged();

  void recalculate3DCorners();
};

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CAMERASETTINGS_H
