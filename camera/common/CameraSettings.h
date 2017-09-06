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
#include <sofa/defaulttype/RigidTypes.h>

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
class CameraSettings : public common::ImplicitDataEngine
{
    SOFAOR_CALLBACK_SYSTEM(CameraSettings);
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
        addAlias(&d_3DCorners, "corners_out");
    }

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

	void init();
    void update() {}
    virtual void reinit()
    {
        std::cout << "reinit" << std::endl;
        std::cout << d_glProjection << std::endl;
        ImplicitDataEngine::reinit();
    }

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
	const sofa::helper::vector<double>& getDistortionCoefficients() const;
	/// sets the distortion coefficients
	void setDistortionCoefficients(const sofa::helper::vector<double>& distCoefs);

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

	/// returns the axis skew parameter (Always 0 in this implementation)
	double getAxisSkew() const;
	/// sets the axis skew parameter (Always 0 in this implementation)
	void setAxisSkew(double s);

	/// returns true if the camera is an XRay device
	bool isXRay() const;
	/// sets whether the camera is an XRay device
	void setXRay(bool isXray);

 private:
	sofa::Data<Vec2i> d_imageSize;      ///< Dimensions of the image in pixels
	sofa::Data<double> d_f;             ///< Focal distance
	sofa::Data<Matrix3> d_translate2D;  ///< 2D translation matrix (u0 v0)
	sofa::Data<Matrix3> d_scale2D;  ///< 2D scale matrix (focal opening, fx, fy)
	sofa::Data<sofa::helper::vector<double> >
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

	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector3> >
			d_3DCorners;  ///< [OUTPUT] 3D positions of the Image plane's corners
public:
	sofa::Data<Vector3> d_upVector;  ///< camera's up vector

	sofa::Data<Vector3>
			d_fwdVector;  ///< direction cam -> lookat (where lookat is
										/// often the 3D position of the projected
										/// Principal Point)
	sofa::Data<Vector3>
			d_lookAt;  ///< target position (a point on the direction camPos
								 /// -> fwdVector)
private:
	sofa::Data<bool> d_isXRay;  ///< Whether or not this camera is an XRay
															/// source-detector model (in which case,
	/// in CalibratedCamera, it is necessary to reverse depth when rasterizing
	/// models, to ensure that the view comes from behind the detector towards the
	/// source, opposite to the standard pinhole camera model

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
    void ProjectionMatrixChanged(sofa::core::objectmodel::BaseData*)
	{
		setProjectionMatrix(d_M.getValue());
	}
    void IntrinsicCameraMatrixChanged(sofa::core::objectmodel::BaseData*)
	{
		setIntrinsicCameraMatrix(d_K.getValue());
	}
    void DistortionCoefficientsChanged(sofa::core::objectmodel::BaseData*)
	{
		setDistortionCoefficients(d_distCoefs.getValue());
	}
    void RotationMatrixChanged(sofa::core::objectmodel::BaseData*)
	{
		setRotationMatrix(d_R.getValue());
	}
    void TranslationVectorChanged(sofa::core::objectmodel::BaseData*)
	{
		setPosition(d_t.getValue());
	}
    void ImageSizeChanged(sofa::core::objectmodel::BaseData*)
	{
		setImageSize(d_imageSize.getValue());
	}
    void GLProjectionChanged(sofa::core::objectmodel::BaseData*)
	{
        setGLProjection(d_glProjection.getValue());
	}
    void GLModelviewChanged(sofa::core::objectmodel::BaseData*)
	{
		setGLModelview(d_glModelview.getValue());
	}
    void GLViewportChanged(sofa::core::objectmodel::BaseData*)
	{
		setGLViewport(d_glViewport.getValue());
	}
    void GLZClipChanged(sofa::core::objectmodel::BaseData*)
	{
		setGLZClip(d_zClip.getValue());
	}
    void OrientationChanged(sofa::core::objectmodel::BaseData*)
	{
		setOrientation(d_orientation.getValue());
	}
    void Scale2DChanged(sofa::core::objectmodel::BaseData*)
	{
		set2DScaleMatrix(d_scale2D.getValue());
	}
    void FocalDistanceChanged(sofa::core::objectmodel::BaseData*)
	{
		setFocalDistance(d_f.getValue());
	}
    void Translation2DChanged(sofa::core::objectmodel::BaseData*)
	{
		set2DTranslationMatrix(d_translate2D.getValue());
	}

	void recalculate3DCorners();
};

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CAMERASETTINGS_H
