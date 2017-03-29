#ifndef SOFA_OR_PROCESSOR_CAMERAP_H
#define SOFA_OR_PROCESSOR_CAMERAP_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualManager.h>
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
class CameraProjectionMat : public common::ImplicitDataEngine
{
	typedef defaulttype::RigidTypes::Coord Rigid;

 public:
	SOFA_CLASS(CameraProjectionMat, common::ImplicitDataEngine);

	CameraProjectionMat()
			: d_pts2d1(initData(&d_pts2d1, "pts2d1", "")),
				d_pts2d2(initData(&d_pts2d2, "pts2d2", "")),
				d_pts3d(initData(&d_pts3d, "pts3d", "OpenGL's 4x4 Projection matrix")),
				d_remapdist(initData(&d_remapdist, "remapDist", "Remap distance")),
				d_remapInd(initData(&d_remapInd, "remapInd", "Remap indices")),
				d_projection(initData(
						&d_projection, defaulttype::Mat3x4d(
															 defaulttype::Vec<4, float>(1.0, 0.0, 0.0, 0.0),
															 defaulttype::Vec<4, float>(0.0, 1.0, 0.0, 0.0),
															 defaulttype::Vec<4, float>(0.0, 0.0, 1.0, 0.0)),
						"projectionMatrix", "Projection matrix")),
				d_projectionMatrixOpenGL(
						initData(&d_projectionMatrixOpenGL,
										 defaulttype::Mat4x4d(
												 defaulttype::Vec<4, float>(1.0, 0.0, 0.0, 0.0),
												 defaulttype::Vec<4, float>(0.0, 1.0, 0.0, 0.0),
												 defaulttype::Vec<4, float>(0.0, 0.0, 1.0, 0.0),
												 defaulttype::Vec<4, float>(0.0, 0.0, 0.0, 1.0)),
										 "glProjection", "Projection matrix OpenGl")),
				d_modelViewMatrixOpenGL(
						initData(&d_modelViewMatrixOpenGL,
										 defaulttype::Mat4x4d(
												 defaulttype::Vec<4, float>(1.0, 0.0, 0.0, 0.0),
												 defaulttype::Vec<4, float>(0.0, 1.0, 0.0, 0.0),
												 defaulttype::Vec<4, float>(0.0, 0.0, 1.0, 0.0),
												 defaulttype::Vec<4, float>(0.0, 0.0, 0.0, 1.0)),
										 "glModelview", "ModelView matrix OpenGl")),
				d_zNear(initData(&d_zNear, 1.00, "zNear", "Camera's ZNear")),
				d_zFar(initData(&d_zFar, 1000.00, "zFar", "Camera's ZFar")),
				d_w(initData(&d_w, 1920, "w", "image size")),
				d_h(initData(&d_h, 1080, "h", "image size"))

	{
	}

	~CameraProjectionMat() {}
	void init()
	{
		addInput(&d_pts2d1);
		addInput(&d_pts2d2);
		addInput(&d_pts3d);
		addInput(&d_zNear);
		addInput(&d_zFar);
		addInput(&d_w);
		addInput(&d_h);

		addOutput(&d_projection);
		addOutput(&d_projectionMatrixOpenGL);
		addOutput(&d_modelViewMatrixOpenGL);
		update();
	}

	void update();
	bool poseEstimation(const helper::vector<defaulttype::Vector3>& p3d,
											const helper::vector<defaulttype::Vector2>& p2d1,
											const helper::vector<defaulttype::Vector2>& p2d2,
											unsigned w, unsigned h, defaulttype::Mat3x4d& M);

	void setCamera(unsigned w, unsigned h);
	void setCamera(defaulttype::Vector3 camera_pos, defaulttype::Quat camera_ori,
								 double fx, double fy, double s, double x0, double y0,
								 unsigned w, unsigned h);
	void setCamera(const defaulttype::Mat3x4d& M, unsigned w, unsigned h);

	void remapVectors(const helper::vector<defaulttype::Vector3>& inpos3d,
										const helper::vector<defaulttype::Vector2>& inpos2d,
										std::vector<cv::Point3f>& outpos3d,
										std::vector<cv::Point2f>& outpos2d);
	defaulttype::Vector2 get2DFrom3DPosition(const defaulttype::Vector3& p);

	Data<helper::vector<defaulttype::Vector2> > d_pts2d1;
	Data<helper::vector<defaulttype::Vector2> > d_pts2d2;
	Data<helper::vector<defaulttype::Vector3> > d_pts3d;
	Data<double> d_zNear, d_zFar;
	Data<int> d_w, d_h;

	Data<defaulttype::Mat3x4d> d_projection;
	Data<defaulttype::Mat4x4d> d_projectionMatrixOpenGL;
	Data<defaulttype::Mat4x4d> d_modelViewMatrixOpenGL;
	Data<Rigid> d_cameraPos;
	defaulttype::Vec<5, float> m_intrinsicParameters;
	Rigid m_cameraPos;
	Data<double> d_remapdist;
	Data<helper::vector<defaulttype::Vec2i> > d_remapInd;
};

SOFA_DECL_CLASS(CameraProjectionMat)

int CameraProjectionMatClass =
		core::RegisterObject(
				"Component computing the projection matrix M "
				"from a set of 2D and 3D points")
				.add<CameraProjectionMat>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CAMERAP_H
