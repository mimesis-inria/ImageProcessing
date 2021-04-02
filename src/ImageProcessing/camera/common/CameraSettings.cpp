#include "CameraSettings.h"
#include <SofaCV/SofaCV.h>

#include <SofaBaseVisual/BaseCamera.h>
#include <iomanip>
#include <limits>

namespace sofacv
{
namespace cam
{
SOFA_DECL_CLASS(CameraSettings)

int CameraSettingsClass =
    sofa::core::RegisterObject(
        "Camera settings component whose task is to store and maintain every "
        "camera parameters up to date, and perform the basic projective "
        "transformations")
        .add<CameraSettings>();

sofa::type::Vector2 CameraSettings::get2DFrom3DPosition(
    const Vector3& pt)
{
  const Mat3x4d& M = d_M.getValue();
  double rx = M[0][0] * pt[0] + M[0][1] * pt[1] + M[0][2] * pt[2] + M[0][3];
  double ry = M[1][0] * pt[0] + M[1][1] * pt[1] + M[1][2] * pt[2] + M[1][3];
  double rz = M[2][0] * pt[0] + M[2][1] * pt[1] + M[2][2] * pt[2] + M[2][3];

  return Vector2(rx, ry) * 1.0 / rz;
}

sofa::type::Vector3 CameraSettings::get3DFrom2DPosition(double x,
                                                               double y,
                                                               double f)
{
  Matrix3 K = d_K.getValue();
  Matrix3 R = d_R.getValue();
  Vector3 t = -R * d_t.getValue();

  Matrix3 iK;
  iK.invert(K);

  Vector3 point3D =
      iK * Vector3(x, y, 1) * ((f == -1) ? (d_f.getValue()) : (f));

  return R.transposed() * (point3D - t);
}

sofa::type::Vector3 CameraSettings::get3DFrom2DPosition(const Vector2& p,
                                                               double f)
{
  return get3DFrom2DPosition(p[0], p[1], f);
}

void CameraSettings::getCornersPosition(Vector3& p1, Vector3& p2, Vector3& p3,
                                        Vector3& p4, double f)
{
  if (d_imageSize.getValue().x() == 0 || d_imageSize.getValue().y() == 0)
  {
    msg_error(getName() + "::getCornersPosition()")
        << "Error: trying to project the image's corners while imageSize has "
           "not been set";
    return;
  }

  int w = d_imageSize.getValue().x();
  int h = d_imageSize.getValue().y();

  p1 = get3DFrom2DPosition(0, 0, f);
  p2 = get3DFrom2DPosition(w, 0, f);
  p3 = get3DFrom2DPosition(w, h, f);
  p4 = get3DFrom2DPosition(0, h, f);
}

const sofa::type::Mat3x4d& CameraSettings::getProjectionMatrix() const
{
  return d_M.getValue();
}
void CameraSettings::setProjectionMatrix(const Mat3x4d& M)
{
  d_M.setValue(M);
  decomposeM();
  composeCV();
  composeGL();

  recalculate3DCorners();
}

const sofa::type::Matrix3& CameraSettings::getIntrinsicCameraMatrix()
    const
{
  return d_K.getValue();
}
void CameraSettings::setIntrinsicCameraMatrix(const Matrix3& K, bool update)
{
  d_K.setValue(K);
  if (update)
  {
    decomposeCV();
    composeM();
    composeGL();
  }

  recalculate3DCorners();
}

const sofa::type::vector<double>& CameraSettings::getDistortionCoefficients()
    const
{
  return d_distCoefs.getValue();
}
void CameraSettings::setDistortionCoefficients(
    const sofa::type::vector<double>& distCoefs)
{
  // Nothing to do, distortion coefficients are not (yet?) taken into account in
  // OpenGL
  d_distCoefs.setValue(distCoefs);

  recalculate3DCorners();
}

const sofa::type::Matrix3& CameraSettings::getRotationMatrix() const
{
  return d_R.getValue();
}
void CameraSettings::setRotationMatrix(const Matrix3& R, bool update)
{
  d_R.setValue(R);
  if (update)
  {
    //    decomposeCV();
    composeM();
    composeGL();
  }

  recalculate3DCorners();
}

const sofa::type::Vector3& CameraSettings::getPosition() const
{
  return d_t.getValue();
}
void CameraSettings::setPosition(const Vector3& t, bool update)
{
  d_t.setValue(t);
  if (update)
  {
    //    decomposeCV();
    composeM();
    composeGL();
  }

  recalculate3DCorners();
}

const sofa::type::Vec2i& CameraSettings::getImageSize() const
{
  return d_imageSize.getValue();
}
void CameraSettings::setImageSize(const Vec2i& imgSize, bool update)
{
  d_imageSize.setValue(imgSize);
  if (update)
  {
    composeM();
    composeGL();
  }

  recalculate3DCorners();
}
const sofa::type::Matrix4& CameraSettings::getGLProjection() const
{
  return d_glProjection.getValue();
}
void CameraSettings::setGLProjection(const Matrix4& glProjection)
{
  d_glProjection.setValue(glProjection);
  decomposeGL();
  composeM();

  recalculate3DCorners();
}
const sofa::type::Matrix4& CameraSettings::getGLModelview() const
{
  return d_glModelview.getValue();
}
void CameraSettings::setGLModelview(const Matrix4& glModelview)
{
  d_glModelview.setValue(glModelview);
  decomposeGL();
  composeM();

  recalculate3DCorners();
}

const sofa::type::Vec<4, int>& CameraSettings::getGLViewport() const
{
  return d_glViewport.getValue();
}
void CameraSettings::setGLViewport(const Vector4& glViewport)
{
  d_glViewport.setValue(glViewport);

  recalculate3DCorners();
}

const sofa::type::Vector2& CameraSettings::getGLZClip() const
{
  return d_zClip.getValue();
}
void CameraSettings::setGLZClip(const Vector2& zClip)
{
  d_zClip.setValue(zClip);
  composeGL();

  recalculate3DCorners();
}
const sofa::type::Quaternion& CameraSettings::getOrientation() const
{
  return d_orientation.getValue();
}
void CameraSettings::setOrientation(const Quat& orientation)
{
  d_orientation.setValue(orientation);
  Matrix3 R;
  orientation.toMatrix(R);
  d_R.setValue(R);
  composeM();
  composeGL();

  recalculate3DCorners();
}

const sofa::type::Matrix3& CameraSettings::get2DScaleMatrix() const
{
  return d_scale2D.getValue();
}
void CameraSettings::set2DScaleMatrix(const Matrix3& scale2D)
{
  d_scale2D.setValue(scale2D);
  composeM();
  composeGL();

  recalculate3DCorners();
}
double CameraSettings::getFocalDistance() const { return d_f.getValue(); }
void CameraSettings::setFocalDistance(double f)
{
  // No need to recompose, fz only used for projection operations
  d_f.setValue(f);

  recalculate3DCorners();
}

const sofa::type::Matrix3& CameraSettings::get2DTranslationMatrix() const
{
  return d_translate2D.getValue();
}
void CameraSettings::set2DTranslationMatrix(const Matrix3& translation2D)
{
  d_translate2D.setValue(translation2D);
  composeM();
  composeGL();

  recalculate3DCorners();
}

const sofa::type::vector<sofa::type::Vector3>&
CameraSettings::getCorners() const
{
  return d_3DCorners.getValue();
}

bool CameraSettings::isXRay() const { return d_isXRay.getValue(); }
void CameraSettings::setXRay(bool isXray) { d_isXRay.setValue(isXray); }

void CameraSettings::decomposeK(const Matrix3& K)
{
  if (d_imageSize.getValue().x() == 0 || d_imageSize.getValue().y() == 0)
  {
    msg_error(getName() + "::decomposeP()")
        << "Error: trying to decompose P while imageSize has not been set";
    return;
  }

  Matrix3 translate2D(Vector3(1, 0, K[0][2]), Vector3(0, 1, K[1][2]),
                      Vector3(0, 0, 1));
  Matrix3 scale2D(Vector3(K[0][0], 0, 0), Vector3(0, K[1][1], 0),
                  Vector3(0, 0, 1));

  d_scale2D.setValue(scale2D);
  d_translate2D.setValue(translate2D);
}

void CameraSettings::IntrinsicCameraMatrixChanged()
{
  setIntrinsicCameraMatrix(d_K.getValue());
}

void CameraSettings::RotationMatrixChanged()
{
  setRotationMatrix(d_R.getValue());
}

void CameraSettings::TranslationVectorChanged() { setPosition(d_t.getValue()); }

void CameraSettings::DistortionCoefficientsChanged()
{
  setDistortionCoefficients(d_distCoefs.getValue());
}

void CameraSettings::ProjectionMatrixChanged()
{
  setProjectionMatrix(d_M.getValue());
}

void CameraSettings::ImageSizeChanged()
{
  setImageSize(d_imageSize.getValue());
}

void CameraSettings::GLProjectionChanged()
{
  setGLProjection(d_glProjection.getValue());
}

void CameraSettings::GLModelviewChanged()
{
  setGLModelview(d_glModelview.getValue());
}

void CameraSettings::GLViewportChanged()
{
  setGLViewport(d_glViewport.getValue());
}

void CameraSettings::GLZClipChanged() { setGLZClip(d_zClip.getValue()); }

void CameraSettings::OrientationChanged()
{
  setOrientation(d_orientation.getValue());
}

void CameraSettings::Scale2DChanged()
{
  set2DScaleMatrix(d_scale2D.getValue());
}

void CameraSettings::FocalDistanceChanged()
{
  setFocalDistance(d_f.getValue());
}

void CameraSettings::Translation2DChanged()
{
  set2DTranslationMatrix(d_translate2D.getValue());
}

// Decomposes P (sets f, c, s, camPos)
void CameraSettings::decomposeM()
{
  cv::Mat_<double> M, K, R, t;
  matrix::sofaMat2cvMat(d_M.getValue(), M);

  cv::decomposeProjectionMatrix(M, K, R, t);

  Vector3 _t =
      Vector3(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)) *
      1.0 / t.at<double>(3, 0);

  Matrix3 _K, _R;
  matrix::cvMat2sofaMat(K, _K);
  matrix::cvMat2sofaMat(R, _R);
  decomposeK(_K);
  d_R.setValue(_R);
  d_t.setValue(_t);
}

// Decomposes OpenGL modelview and projection matrix
void CameraSettings::decomposeGL()
{
  Matrix4 glP = d_glProjection.getValue();
  Matrix4 glM = d_glModelview.getValue();

  double w = d_imageSize.getValue().x();
  double h = d_imageSize.getValue().y();

  double fx = 0.5 * w * glP[0][0];
  double fy = 0.5 * h * glP[1][1];
  double cx = 0.5 * w * (1.0 - glP[0][2]);
  double cy = 0.5 * h * (1.0 + glP[1][2]);

  Matrix3 translate2D(Vector3(1, 0, cx), Vector3(0, 1, cy), Vector3(0, 0, 1));
  Matrix3 scale2D(Vector3(fx, 0, 0), Vector3(0, fy, 0), Vector3(0, 0, 1));

  d_translate2D.setValue(translate2D);
  d_scale2D.setValue(scale2D);

  Matrix3 R;
  for (unsigned j = 0; j < 3; j++)
    for (unsigned i = 0; i < 3; i++) R[j][i] = glM[i][j];

  Quat camera_ori;
  camera_ori.fromMatrix(R.transposed());
  Vector3 camera_pos(glM[0][3], glM[1][3], glM[2][3]);
  camera_pos = -R * camera_pos;

  camera_ori = Quat(Vector3(1.0, 0.0, 0.0), M_PI) * camera_ori;
  camera_ori.toMatrix(R);

  d_orientation.setValue(camera_ori);
  d_R.setValue(R);
  d_t.setValue(camera_pos);
}

// decomposes OpenCV's K, R and t
void CameraSettings::decomposeCV() { decomposeK(d_K.getValue()); }

// Composes OpenGL's Modelview and Projection matrices
void CameraSettings::composeGL()
{
  double n = d_zClip.getValue().x();
  double f = d_zClip.getValue().y();
  double w = d_imageSize.getValue().x();
  double h = d_imageSize.getValue().y();

  Matrix3 T = d_translate2D.getValue();
  Matrix3 S = d_scale2D.getValue();

  Matrix4 glP;

  glP[0][0] = 2.0 * S[0][0] / w;
  glP[0][1] = 0.0;
  glP[0][2] = 1.0 - 2.0 * T[0][2] / w;
  glP[0][3] = 0.0;

  //	glP[1][0] = -2.0 * s / w;
  glP[1][0] = 0.0;
  glP[1][1] = 2.0 * S[1][1] / h;
  glP[1][2] = -1.0 + 2.0 * T[1][2] / h;
  glP[1][3] = 0.0;

  glP[2][0] = 0.0;
  glP[2][1] = 0.0;
  glP[2][2] = (-(f + n)) / (f - n);
  glP[2][3] = (-2.0 * f * n) / (f - n);

  glP[3][0] = 0.0;
  glP[3][1] = 0.0;
  glP[3][2] = -1.0;
  glP[3][3] = 0.0;

  d_glProjection.setValue(glP);

  Matrix4 MM;

  Quat q;
  q.fromMatrix(d_R.getValue());
  q = Quat(Vector3(1.0, 0.0, 0.0), M_PI) * q;
  d_orientation.setValue(q);

  Matrix3 R;
  d_orientation.getValue().toMatrix(R);

  Vector3 p = -R * d_t.getValue();

  for (unsigned int j = 0; j < 3; j++)
  {
    for (unsigned int i = 0; i < 3; i++) MM[j][i] = R[j][i];

    MM[j][3] = p[j];
    MM[3][j] = 0.0;
  }
  MM[3][3] = 1.0;

  d_glModelview.setValue(MM);
}

// Composes K, R, t
void CameraSettings::composeCV()
{
  Matrix3 T = d_translate2D.getValue();
  Matrix3 S = d_scale2D.getValue();

  Matrix3 K = T * S;
  d_K.setValue(K);
}

// Composes the Projection matrix P
void CameraSettings::composeM()
{
  composeCV();
  Vector3 t = d_t.getValue();
  const Matrix3& R = d_R.getValue();
  const Matrix3& K = d_K.getValue();

  // gets the camera position from the position of the world's reference frame
  // in camera coordinates
  t = -R * t;

  double ptr2[12] = {R[0][0], R[0][1], R[0][2], t[0],    R[1][0], R[1][1],
                     R[1][2], t[1],    R[2][0], R[2][1], R[2][2], t[2]};
  Mat3x4d Rt(ptr2);

  Mat3x4d M = K * Rt;

  d_M.setValue(M);

  std::string path = "poses.txt";
  std::string opath = "%03d_test.png";
  file.open(path.c_str(), std::ios_base::app);
  double dvalue0, dvalue1, dvalue2, dvalue3, dvalue4, dvalue5, dvalue6;
  // for (int k = 0; k < pcd.size(); k++)
  // if ((index0-1)%3==0)
  {
    char buf4[FILENAME_MAX];
    sprintf(buf4, opath.c_str(), (index0));
    std::string filename4(buf4);
    dvalue0 = t[0] / 100.0;
    dvalue1 = t[1] / 100.0;
    dvalue2 = t[2] / 100.0;
    dvalue3 = (this->getOrientation())[0];
    dvalue4 = (this->getOrientation())[1];
    dvalue5 = (this->getOrientation())[2];
    dvalue6 = (this->getOrientation())[3];

    file << filename4;
    file << " ";
    file << dvalue0;
    file << " ";
    file << dvalue1;
    file << " ";
    file << dvalue2;
    file << " ";
    file << dvalue3;
    file << " ";
    file << dvalue4;
    file << " ";
    file << dvalue5;
    file << " ";
    file << dvalue6;
    file << " ";
    file << "\n";
  }
  std::cout << " index " << index0 << std::endl;
  std::cout << " camera " << getIntrinsicCameraMatrix() << std::endl;
  std::cout << " Rt " << Rt << std::endl;
  std::cout << " R " << getRotationMatrix() << std::endl;
  std::cout << " t " << getPosition() << std::endl;
  std::cout << " image size " << d_imageSize.getValue().x() << " "
            << d_imageSize.getValue().y() << std::endl;
  index0++;
  file.close();
}

void CameraSettings::recalculate3DCorners()
{
  if (d_K.getValue() == Matrix3()) return;
  Vector3 p1, p2, p3, p4;
  this->getCornersPosition(p1, p2, p3, p4,
                           (d_f.getValue() != -1) ? (d_f.getValue()) : (1.0f));
  sofa::type::vector<Vector3>& corners3D = *d_3DCorners.beginEdit();
  corners3D.clear();
  corners3D.push_back(p1);
  corners3D.push_back(p2);
  corners3D.push_back(p3);
  corners3D.push_back(p4);
  d_3DCorners.endEdit();
}

/// Projects a point p on a plane defined by a Point A and a normal n
sofa::type::Vector3 orthoProj(const sofa::type::Vector3& p,
                                     const sofa::type::Vector3& A,
                                     const sofa::type::Vector3& n)
{
  double lambda = ((A * n) - (p * n)) / (n * n);
  return p + lambda * n;
}

double length(const sofa::type::Vector3& a,
              const sofa::type::Vector3& b)
{
  return sqrt((a.x() - b.x()) * (a.x() - b.x()) +
              (a.y() - b.y()) * (a.y() - b.y()) +
              (a.z() - b.z()) * (a.z() - b.z()));
}

void CameraSettings::buildFromCamPosAndImageCorners()
{
  double w = d_imageSize.getValue().x();
  double h = d_imageSize.getValue().x();

  Vector3 A = d_3DCorners.getValue()[0];
  Vector3 B = d_3DCorners.getValue()[1];
  Vector3 D = d_3DCorners.getValue()[3];

  Vector3 c = d_t.getValue();

  double u0, v0, fu, fv, f, Xmm, Ymm;
  Vector3 O, n, AB, AD, x, y;

  AB = B - A;
  AD = D - A;
  n = AB.normalized().cross(AD.normalized());

  // Optical center projection
  O = orthoProj(c, A, n);

  // focal distance
  f = length(c, O);

  x = orthoProj(O, A, AD.normalized());
  y = orthoProj(O, A, AB.normalized());

  double xdist = length(x, A);
  double ydist = length(y, A);

  // image dimensions in world units
  Xmm = length(B, A);
  Ymm = length(D, A);

  // pixel size
  double Su = Xmm / w;
  double Sv = Ymm / h;

  // 2D scaling
  fu = f / Su;
  fv = f / Sv;

  u0 = xdist / Su;
  v0 = ydist / Sv;

  Matrix3 R(Vector3(AB.normalized()), Vector3(AD.normalized()), Vector3(n));

  Rigid cam;
  cam.getOrientation().fromMatrix(R);
  cam.getCenter() = c;

  d_f = f;
  d_translate2D =
      Matrix3(Vector3(1, 0, u0), Vector3(0, 1, v0), Vector3(0, 0, 1));
  d_scale2D = Matrix3(Vector3(fu, 0, 0), Vector3(0, fv, 0), Vector3(0, 0, 1));

  composeCV();
  composeM();
  composeGL();
}

CameraSettings::CameraSettings()
    : d_imageSize(
          initData(&d_imageSize, "imageSize", "Image resolution in pixels")),
      d_f(initData(&d_f, 1.0, "f", "distance camera -> plane")),
      d_translate2D(
          initData(&d_translate2D, "translate2D",
                   "principal point position in the image (in pixel units)")),
      d_scale2D(initData(&d_scale2D, "scale2D", "focal opening")),
      d_distCoefs(initData(&d_distCoefs, "distCoefs",
                           "The camera's distortion coefficients")),

      d_M(initData(&d_M, "M",
                   "3x4 Projection matrix as described in Computer Vision")),

      d_K(initData(&d_K, "K", "3x3 camera matrix from OpenCV")),
      d_R(initData(&d_R, "R", "3x3 rotation matrix")),
      d_t(initData(&d_t, "position",
                   "optical center position in the world reference frame")),
      d_glProjection(initData(&d_glProjection, "glProjection",
                              "OpenGL's 4x4 Projection matrix")),
      d_glModelview(initData(&d_glModelview, "glModelview",
                             "OpenGL's 4x4 Modelview matrix")),
      d_orientation(initData(&d_orientation, "orientation",
                             "OpenGL's camera orientation z-axis flipped")),
      d_glViewport(initData(&d_glViewport, "glViewport", "OpenGL's Viewport")),
      d_zClip(initData(&d_zClip, Vector2(0.001, 1000.0), "zClip",
                       "OpenGL's z clipping values in scene unit")),
      d_3DCorners(initData(&d_3DCorners, "3DCorners",
                           "image's corners in world coordinates")),
      d_upVector(initData(&d_upVector, "up", "Camera's Up vector")),
      d_fwdVector(initData(&d_fwdVector, "fwd", "Camera's lookat direction")),
      d_lookAt(initData(&d_lookAt, "lookAt", "Camera's lookat point")),
      d_isXRay(initData(&d_isXRay, "isXRay",
                        "whether or not the camera model is an XRay model as "
                        "opposite to the standard pinhole model"))
{
  addAlias(&d_t, "t");
  addAlias(&d_3DCorners, "corners");
  addAlias(&d_3DCorners, "corners_out");
}

void CameraSettings::buildFromIntrinsicCamPosLookAtAndUpVector()
{
  Vector3 up, fwd, right;
  fwd = d_lookAt.getValue() - d_t.getValue();
  fwd.normalize();

  up = d_upVector.getValue().normalized();
  right = up.cross(fwd).normalized();

  Matrix3 R(right.normalized(), up.normalized(), fwd.normalized());
  //			R.transpose();
  d_R.setValue(R);

  decomposeCV();
  composeM();
  composeGL();
}

void CameraSettings::buildFromIntrinsicCamPosUpVectorAndFwdVector()
{
  std::cout << "setting calibration from K, t, up and lookAt" << std::endl;
  Vector3 right(d_upVector.getValue().normalized().cross(
      d_fwdVector.getValue().normalized()));
  Matrix3 R(right.normalized(), d_fwdVector.getValue().normalized(),
            d_upVector.getValue().normalized());
  //			R.transpose();
  d_R.setValue(R);

  decomposeCV();
  composeM();
  composeGL();
}

void CameraSettings::buildFromM()
{
  decomposeM();
  composeCV();
  composeGL();
}

void CameraSettings::buildFromKRT()
{
  decomposeCV();
  composeM();
  composeGL();
  sofa::type::Matrix3 rot;
  sofa::type::Quat q = getOrientation();
  q.toMatrix(rot);

  Vector3 camPos = getPosition();
  Vector3 camera_Y = rot.line(1).normalized();
  Vector3 camera_Z = rot.line(2).normalized();
  d_upVector.setValue(camera_Y);
  d_fwdVector.setValue(camera_Z);
  d_lookAt.setValue(camPos - (camera_Z * d_f.getValue()));
}

void CameraSettings::buildFromOpenGL()
{
  decomposeGL();
  composeM();
}

void CameraSettings::buildFromOpenGLContext()
{
  Matrix4 p, m;
  std::cout << "initializing from Opengl's default settings" << std::endl;
  glGetDoublev(GL_PROJECTION_MATRIX, p.ptr());
  glGetDoublev(GL_MODELVIEW_MATRIX, m.ptr());

  bool hasGLContext = false;
  for (int i = 0; i < 16; ++i)
  {
    if (std::fabs(p.ptr()[0]) > std::numeric_limits<double>::epsilon())
      hasGLContext = true;
  }
  if (hasGLContext)
  {
    d_glProjection.setValue(p.transposed());
    d_glModelview.setValue(m.transposed());
  }
  else
  {
    d_glProjection.setValue(Matrix4::s_identity);
    d_glModelview.setValue(Matrix4::s_identity);
  }
  buildFromOpenGL();
}

void CameraSettings::reinit()
{
  if (m_dataTracker.hasChanged(d_M)) ProjectionMatrixChanged();
  if (m_dataTracker.hasChanged(d_K)) IntrinsicCameraMatrixChanged();
  if (m_dataTracker.hasChanged(d_distCoefs)) DistortionCoefficientsChanged();
  if (m_dataTracker.hasChanged(d_R)) RotationMatrixChanged();
  if (m_dataTracker.hasChanged(d_t)) TranslationVectorChanged();
  if (m_dataTracker.hasChanged(d_imageSize)) ImageSizeChanged();
  if (m_dataTracker.hasChanged(d_glProjection)) GLProjectionChanged();
  if (m_dataTracker.hasChanged(d_glModelview)) GLModelviewChanged();
  if (m_dataTracker.hasChanged(d_glViewport)) GLViewportChanged();
  if (m_dataTracker.hasChanged(d_zClip)) GLZClipChanged();
  if (m_dataTracker.hasChanged(d_orientation)) OrientationChanged();
  if (m_dataTracker.hasChanged(d_scale2D)) Scale2DChanged();
  if (m_dataTracker.hasChanged(d_translate2D)) Translation2DChanged();
  if (m_dataTracker.hasChanged(d_f)) FocalDistanceChanged();
}

void CameraSettings::doUpdate() {}

void CameraSettings::init()
{
  m_dataTracker.trackData(d_M);
  m_dataTracker.trackData(d_K);
  m_dataTracker.trackData(d_distCoefs);
  m_dataTracker.trackData(d_R);
  m_dataTracker.trackData(d_t);
  m_dataTracker.trackData(d_imageSize);
  m_dataTracker.trackData(d_glProjection);
  m_dataTracker.trackData(d_glModelview);
  m_dataTracker.trackData(d_glViewport);
  m_dataTracker.trackData(d_zClip);
  m_dataTracker.trackData(d_orientation);
  m_dataTracker.trackData(d_scale2D);
  m_dataTracker.trackData(d_f);
  m_dataTracker.trackData(d_translate2D);

  addOutput(&d_3DCorners);

  if (!d_upVector.isSet()) addOutput(&d_upVector);
  if (!d_fwdVector.isSet()) addOutput(&d_fwdVector);
  if (!d_lookAt.isSet()) addOutput(&d_lookAt);

  if (!d_imageSize.isSet()) d_imageSize.setValue(Vec2i(1280, 720));

  if (d_t.isSet() && d_K.isSet() && d_upVector.isSet() && d_lookAt.isSet())
  {
    buildFromIntrinsicCamPosLookAtAndUpVector();
  }
  else if (d_t.isSet() && d_K.isSet() && d_upVector.isSet() &&
           d_fwdVector.isSet())
  {
    buildFromIntrinsicCamPosUpVectorAndFwdVector();
  }
  else if (d_3DCorners.isSet() && d_t.isSet())
  {
    this->buildFromCamPosAndImageCorners();
  }
  else if (d_M.isSet())
  {
    buildFromM();
  }
  else if (d_K.isSet() && (d_R.isSet() || d_t.isSet()))
  {
    buildFromKRT();
  }
  else if (d_glProjection.isSet() || d_glModelview.isSet())
  {
    buildFromOpenGL();
  }
  else
  {
    buildFromOpenGLContext();
  }
  recalculate3DCorners();
}

}  // namespace cam
}  // namespace sofacv
