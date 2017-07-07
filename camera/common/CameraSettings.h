#ifndef SOFA_OR_PROCESSOR_CAMERASETTINGS_H
#define SOFA_OR_PROCESSOR_CAMERASETTINGS_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualManager.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/gl/Transformation.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glu.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
/**
 * \brief Camera-related stuff
 */
namespace cam
{
class CameraSettings : public common::ImplicitDataEngine
{
	/// This component holds the monoscopic camera parameters and maintain their
	/// 4 different representations up-to-date at all time:
	/// - OpenGL representation (4x4 modelview + projection matrix)
	/// - "Vision" representation: (3x4 projection matrix M)
	/// - "OpenCV" representation: (3x3 K, R, and vector t)
	/// - "Decomposed" representation: 2D scale, skew and translation matrix,
	///    camera position, orientation etc.
	/// There is multiple ways of initializing this component, but the imageSize
	/// and focal distance should always be provided. Their default value are
	/// 1280x720 and 1.0 and can be left that way if satisfying.
	/// - By simply creating the component with its default parameters,
	///   CameraSettings will initialize itself according to OpenGL's current
	///   modelview and projection matrix.
	/// - One can also set the right parameters a-posteriori using another
	///   component such as SolvePnP or CalibrateCamera
	/// - A camera can be defined using only 5 points (technically 4 but here
	///   5...): the position of the camera in world coordinates (C), and the
	///   position of the 4 corners of the image plane in world coordinates
	///   (corners3D)
	/// - otherwise the camera settings will be extracted from either:
	/// - M if provided
	/// - K, R and t if provided
	/// - glModelview, glProjection if provided
	/// - The Opengl modelview / projection matrix currently set in OpenGL's
	///   context

 public:
	typedef sofa::defaulttype::RigidTypes::Coord Rigid;
	typedef sofa::defaulttype::Vector2 Vector2;
	typedef sofa::defaulttype::Vector3 Vector3;
	typedef sofa::defaulttype::Vec<4, int> Vector4;
	typedef sofa::defaulttype::Vec<2, int> Vec2i;
	typedef sofa::defaulttype::Vec<5, double> Vector5;
	typedef sofa::defaulttype::Mat3x4d Mat3x4d;
	typedef sofa::defaulttype::Matrix4 Matrix4;
	typedef sofa::defaulttype::Matrix3 Matrix3;
	typedef sofa::defaulttype::Quat Quat;

 public:
	SOFA_CLASS(CameraSettings, common::ImplicitDataEngine);

	CameraSettings()
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
				d_glViewport(
						initData(&d_glViewport, "glViewport", "OpenGL's Viewport")),
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
	}

	~CameraSettings() {}

	void buildFromCamPosAndImageCorners();

	void init();
	void update() { checkData(true); }

	// returns the 2D pixel position of a given 3D point
	Vector2 get2DFrom3DPosition(const Vector3& p);

	// returns the 3D position of a 2D point 'x, y'
	Vector3 get3DFrom2DPosition(double x, double y, double fz = -1);
	Vector3 get3DFrom2DPosition(const Vector2& p, double fz = -1);

	// Returns the corners of the image plane
	void getCornersPosition(Vector3& p1, Vector3& p2, Vector3& p3, Vector3& p4,
													double fz = -1);

	// Getters & setters for the private Data (also used as callbacks when
	// modifying values in the GUI
	const Mat3x4d& getProjectionMatrix() const;
	void setProjectionMatrix(const Mat3x4d& M);

	const Matrix3& getIntrinsicCameraMatrix() const;
	void setIntrinsicCameraMatrix(const Matrix3& K, bool update = true);

	const sofa::helper::vector<double>& getDistortionCoefficients() const;
	void setDistortionCoefficients(const sofa::helper::vector<double>& distCoefs);

	const Matrix3& getRotationMatrix() const;
	void setRotationMatrix(const Matrix3& R, bool update = true);

	const Vector3& getPosition() const;
	void setPosition(const Vector3& t, bool update = true);

	const Vec2i& getImageSize() const;
	void setImageSize(const Vec2i& imgSize, bool update = true);
	const Matrix4& getGLProjection() const;
	void setGLProjection(const Matrix4& glProjection);

	const Matrix4& getGLModelview() const;
	void setGLModelview(const Matrix4& glModelview);

	const Vector4& getGLViewport() const;
	void setGLViewport(const Vector4& glViewport);

	const Vector2& getGLZClip() const;
	void setGLZClip(const Vector2& zClip);

	const Quat& getOrientation() const;
	void setOrientation(const Quat& camPos);

	const Matrix3& get2DScaleMatrix() const;
	void set2DScaleMatrix(const Matrix3& scale2DMat);

	double getFocalDistance() const;
	void setFocalDistance(double f);

	const Matrix3& get2DTranslationMatrix() const;
	void set2DTranslationMatrix(const Matrix3& translation2DMat);

	double getAxisSkew() const;
	void setAxisSkew(double s);

	bool isXRay() const;
	void setXRay(bool isXray);

 private:
	// Dimensions in pixels of the image
	sofa::Data<Vec2i> d_imageSize;
	// Focal distance
	sofa::Data<double> d_f;

	// 2D translation matrix (u0 v0)
	sofa::Data<Matrix3> d_translate2D;
	// 2D scale matrix (focal opening)
	sofa::Data<Matrix3> d_scale2D;
	// Distortion coefficients
	sofa::Data<sofa::helper::vector<double> > d_distCoefs;

	// 3x4 global projection matrix
	sofa::Data<Mat3x4d> d_M;

	// Intrinsic camera matrix
	sofa::Data<Matrix3> d_K;
	// 3x3 rotation matrix
	sofa::Data<Matrix3> d_R;
	// Position in world coordinates of the camera's optical center
	sofa::Data<Vector3> d_t;

	// 4x4 Opengl Projection matrix
	sofa::Data<Matrix4> d_glProjection;
	// 4x4 Opengl Modelview matrix
	sofa::Data<Matrix4> d_glModelview;
	// Quaternion representation of the Rotation Matrix
	sofa::Data<Quat> d_orientation;
	// Opengl Viewport
	sofa::Data<Vector4> d_glViewport;
	// zNear, zFar
	sofa::Data<Vector2> d_zClip;

	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector3> > d_3DCorners;

	// Camera can be constructed with K, t, upVector and fwdVector
	sofa::Data<Vector3> d_upVector;

	// Camera can be constructed with K, t, upVector and lookAt
	sofa::Data<Vector3> d_fwdVector;  // direction cam -> lookat
	sofa::Data<Vector3> d_lookAt;  // target position (a point on the direction camPos
													 // -> fwdVector)

	// Whether or not this camera is an XRay source-detector model (in which case,
	// in CalibratedCamera, it is necessary to reverse depth when rasterizing
	// models, to ensure that the view comes from behind the detector towards the
	// source, opposite to the standard pinhole camera model
	sofa::Data<bool> d_isXRay;

	// Decomposes M
	void decomposeM();

	// Decomposes OpenGL modelview and projection matrix
	void decomposeGL();

	// decomposes OpenCV's K, R and t
	void decomposeCV();

	// Composes OpenGL's Modelview and Projection matrices
	void composeGL();

	// Composes K, R, t and camPos
	void composeCV();

	// Composes the Projection matrix M
	void composeM();
	void decomposeK(const Matrix3& K);

 public:
	// Data callbacks for GUI
	void ProjectionMatrixChanged(sofa::core::objectmodel::BaseObject*)
	{
		setProjectionMatrix(d_M.getValue());
	}
	void IntrinsicCameraMatrixChanged(sofa::core::objectmodel::BaseObject*)
	{
		setIntrinsicCameraMatrix(d_K.getValue());
	}
	void DistortionCoefficientsChanged(sofa::core::objectmodel::BaseObject*)
	{
		setDistortionCoefficients(d_distCoefs.getValue());
	}
	void RotationMatrixChanged(sofa::core::objectmodel::BaseObject*)
	{
		setRotationMatrix(d_R.getValue());
	}
	void TranslationVectorChanged(sofa::core::objectmodel::BaseObject*)
	{
		setPosition(d_t.getValue());
	}
	void ImageSizeChanged(sofa::core::objectmodel::BaseObject*)
	{
		setImageSize(d_imageSize.getValue());
	}
	void GLProjectionChanged(sofa::core::objectmodel::BaseObject*)
	{
		setGLProjection(d_glProjection.getValue());
	}
	void GLModelviewChanged(sofa::core::objectmodel::BaseObject*)
	{
		setGLModelview(d_glModelview.getValue());
	}
	void GLViewportChanged(sofa::core::objectmodel::BaseObject*)
	{
		setGLViewport(d_glViewport.getValue());
	}
	void GLZClipChanged(sofa::core::objectmodel::BaseObject*)
	{
		setGLZClip(d_zClip.getValue());
	}
	void OrientationChanged(sofa::core::objectmodel::BaseObject*)
	{
		setOrientation(d_orientation.getValue());
	}
	void Scale2DChanged(sofa::core::objectmodel::BaseObject*)
	{
		set2DScaleMatrix(d_scale2D.getValue());
	}
	void FocalDistanceChanged(sofa::core::objectmodel::BaseObject*)
	{
		setFocalDistance(d_f.getValue());
	}
	void Translation2DChanged(sofa::core::objectmodel::BaseObject*)
	{
		set2DTranslationMatrix(d_translate2D.getValue());
	}

	void recalculate2DCorners();
	void recalculate3DCorners();
};

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CAMERASETTINGS_H
