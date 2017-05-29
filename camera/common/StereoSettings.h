#ifndef SOFA_OR_PROCESSOR_STEREOSETTINGS_H
#define SOFA_OR_PROCESSOR_STEREOSETTINGS_H

#include "initPlugin.h"
#include "CameraSettings.h"

#include <SofaORCommon/ImplicitDataEngine.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class StereoSettings : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<StereoSettings, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	typedef defaulttype::Vector3 Vector3;
	typedef defaulttype::Vector2 Vector2;
	typedef defaulttype::Matrix3 Matrix3;

 public:
	SOFA_CLASS(StereoSettings, common::ImplicitDataEngine);

	// StereoSettings ctor. All parameters are optional and can be set using
	// calibration components or file loaders
	StereoSettings()
			: l_cam1(
						initLink("cam1", "link to the reference CameraSettings component")),
				l_cam2(initLink("cam2", "link to the second CameraSettings component")),
				d_F(initData(&d_F, "F", "Fundamental matrix")),
				d_E(initData(&d_E, "E", "Essential matrix"))
//				d_R(initData(&d_R, "R", "Rotation matrix from camera1 to camera2")),
//				d_t(initData(&d_t, "t", "tranlsation vector from camera1 to camera2"))
	{
	}

	~StereoSettings() {}
	void init();
	void update() {}
	// returns the 3D position of a pair of 2D matches 'X, Y'
	Vector3 triangulate(const Vector2& x1, const Vector2& x2);
	// returns the 3D position of a pair of 2D matches 'X, Y'
	void triangulate(const Vector2& x1, const Vector2& x2, Vector3& w);

	// Getters & setters for the private Data (also used as callbacks when
	// modifying values in the GUI
	const Matrix3& getFundamentalMatrix();
	void setFundamentalMatrix(const Matrix3& F);

	const Matrix3& getEssentialMatrix();
	void setEssentialMatrix(const Matrix3& E);

//	const Matrix3& getRotationMatrix();
//	void setRotationMatrix(const Matrix3& R);

//	const Vector3& getTranslationVector();
//	void setTranslationVector(const Vector3& t);

	const CameraSettings& getCamera1() { return *l_cam1.get(); }
	const CameraSettings& getCamera2() { return *l_cam2.get(); }

	void recomputeFromCameras();
 private:
	CamSettings l_cam1;
	CamSettings l_cam2;
	Data<Matrix3> d_F;
	Data<Matrix3> d_E;
//	Data<Matrix3> d_R;
//	Data<Vector3> d_t;

//	cv::Mat_<double> K1, K2;
	cv::Mat_<double> P1, P2;

 public:
	// Data callbacks for GUI
	void FundamentalMatrixChanged(core::objectmodel::BaseObject*)
	{
		setFundamentalMatrix(d_F.getValue());
		this->checkData(false);
	}
	void EssentialMatrixChanged(core::objectmodel::BaseObject*)
	{
		setEssentialMatrix(d_E.getValue());
		this->checkData(false);
	}
//	void RotationMatrixChanged(core::objectmodel::BaseObject*)
//	{
//		setRotationMatrix(d_R.getValue());
//		this->checkData(false);
//	}
//	void TranslationVectorChanged(core::objectmodel::BaseObject*)
//	{
//		setTranslationVector(d_t.getValue());
//		this->checkData(false);
//	}

 private:
	cv::Mat_<double> iterativeLinearLSTriangulation(cv::Point3d u,
																									cv::Point3d u1);
	cv::Mat_<double> linearLSTriangulation(cv::Point3d u, cv::Point3d u1);
	void updateRt();
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_STEREOSETTINGS_H
