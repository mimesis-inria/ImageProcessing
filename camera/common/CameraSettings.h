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

namespace sofa
{
namespace OR
{
namespace processor
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
	typedef defaulttype::RigidTypes::Coord Rigid;
	typedef defaulttype::Vector2 Vector2;
	typedef defaulttype::Vector3 Vector3;
	typedef defaulttype::Vec<4, int> Vector4;
	typedef defaulttype::Vec<2, int> Vec2i;
	typedef defaulttype::Vec<5, double> Vector5;
	typedef defaulttype::Mat3x4d Mat3x4d;
	typedef defaulttype::Matrix4 Matrix4;
	typedef defaulttype::Matrix3 Matrix3;
	typedef defaulttype::Quat Quat;

 public:
	SOFA_CLASS(CameraSettings, common::ImplicitDataEngine);

	CameraSettings()
			: d_imageSize(
						initData(&d_imageSize, Vec2i(1280, 720), "imageSize", "Image resolution in pixels")),
				d_f(initData(&d_f, "f", "distance camera -> plane")),
				d_translate2D(
						initData(&d_translate2D, "translate2D",
										 "principal point position in the image (in pixel units)")),
				d_scale2D(initData(&d_scale2D, "scale2D", "focal opening")),
				d_distCoefs(initData(&d_distCoefs, "distCoefs",
														 "The camera's distortion coefficients")),

				d_M(initData(&d_M, "M", "3x4 Projection matrix as described in Computer Vision")),

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
														 "image's corners in world coordinates"))
	{
		addAlias(&d_t, "t");
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

	const helper::vector<double>& getDistortionCoefficients() const;
	void setDistortionCoefficients(const helper::vector<double>& distCoefs);

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

 private:
	// Dimensions in pixels of the image
	Data<Vec2i> d_imageSize;
	// Focal distance
	Data<double> d_f;

	// 2D translation matrix (u0 v0)
	Data<Matrix3> d_translate2D;
	// 2D scale matrix (focal opening)
	Data<Matrix3> d_scale2D;
	// Distortion coefficients
	Data<helper::vector<double> > d_distCoefs;

	// 3x4 global projection matrix
	Data<Mat3x4d> d_M;

	// Intrinsic camera matrix
	Data<Matrix3> d_K;
	// 3x3 rotation matrix
	Data<Matrix3> d_R;
	// Position in world coordinates of the camera's optical center
	Data<Vector3> d_t;

	// 4x4 Opengl Projection matrix
	Data<Matrix4> d_glProjection;
	// 4x4 Opengl Modelview matrix
	Data<Matrix4> d_glModelview;
	// Quaternion representation of the Rotation Matrix
	Data<Quat> d_orientation;
	// Opengl Viewport
	Data<Vector4> d_glViewport;
	// zNear, zFar
	Data<Vector2> d_zClip;

	Data<helper::vector<defaulttype::Vector3> > d_3DCorners;

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
	void ProjectionMatrixChanged(core::objectmodel::BaseObject*)
	{
		setProjectionMatrix(d_M.getValue());
	}
	void IntrinsicCameraMatrixChanged(core::objectmodel::BaseObject*)
	{
		setIntrinsicCameraMatrix(d_K.getValue());
	}
	void DistortionCoefficientsChanged(core::objectmodel::BaseObject*)
	{
		setDistortionCoefficients(d_distCoefs.getValue());
	}
	void RotationMatrixChanged(core::objectmodel::BaseObject*)
	{
		setRotationMatrix(d_R.getValue());
	}
	void TranslationVectorChanged(core::objectmodel::BaseObject*)
	{
		setPosition(d_t.getValue());
	}
	void ImageSizeChanged(core::objectmodel::BaseObject*)
	{
		setImageSize(d_imageSize.getValue());
	}
	void GLProjectionChanged(core::objectmodel::BaseObject*)
	{
		setGLProjection(d_glProjection.getValue());
	}
	void GLModelviewChanged(core::objectmodel::BaseObject*)
	{
		setGLModelview(d_glModelview.getValue());
	}
	void GLViewportChanged(core::objectmodel::BaseObject*)
	{
		setGLViewport(d_glViewport.getValue());
	}
	void GLZClipChanged(core::objectmodel::BaseObject*)
	{
		setGLZClip(d_zClip.getValue());
	}
	void OrientationChanged(core::objectmodel::BaseObject*)
	{
		setOrientation(d_orientation.getValue());
	}
	void Scale2DChanged(core::objectmodel::BaseObject*)
	{
		set2DScaleMatrix(d_scale2D.getValue());
	}
	void FocalDistanceChanged(core::objectmodel::BaseObject*)
	{
		setFocalDistance(d_f.getValue());
	}
	void Translation2DChanged(core::objectmodel::BaseObject*)
	{
		set2DTranslationMatrix(d_translate2D.getValue());
	}
	void recalculate2DCorners();
	void recalculate3DCorners();
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMERASETTINGS_H
