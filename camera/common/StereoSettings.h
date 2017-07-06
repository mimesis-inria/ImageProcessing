#ifndef SOFA_OR_PROCESSOR_STEREOSETTINGS_H
#define SOFA_OR_PROCESSOR_STEREOSETTINGS_H

#include "CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
class StereoSettings : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<StereoSettings, CameraSettings,
																							sofa::BaseLink::FLAG_STOREPATH |
																									sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	typedef sofa::defaulttype::Vector3 Vector3;
	typedef sofa::defaulttype::Vector2 Vector2;
	typedef sofa::defaulttype::Matrix3 Matrix3;

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
	//				d_R(initData(&d_R, "R", "Rotation matrix from camera1
	//to
	// camera2")),
	//				d_t(initData(&d_t, "t", "tranlsation vector from camera1
	//to
	// camera2"))
	{
	}

	~StereoSettings() {}
	void init();
	void update() {}
	// returns the 3D position of a pair of 2D matches 'X, Y'
	Vector3 triangulate(const Vector2& x1, const Vector2& x2);
	Vector3 triangulate(const cv::Point2d& x1, const cv::Point2d& x2);
	// returns the 3D position of
	// a pair of 2D matches 'X, Y'
	void triangulate(const Vector2& x1, const Vector2& x2, Vector3& w);
	void triangulate(const cv::Point2d& x1, const cv::Point2d& x2,
									 Vector3& w);

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

	CameraSettings& getCamera1() { return *l_cam1.get(); }
	CameraSettings& getCamera2() { return *l_cam2.get(); }

	void recomputeFromCameras();

 private:
	CamSettings l_cam1;
	CamSettings l_cam2;
	sofa::Data<Matrix3> d_F;
	sofa::Data<Matrix3> d_E;

	//	cv::Mat_<double> K1, K2;
	cv::Mat_<double> P1, P2;

 public:
	// Data callbacks for GUI
	void FundamentalMatrixChanged(sofa::core::objectmodel::BaseObject*)
	{
		setFundamentalMatrix(d_F.getValue());
		this->checkData(false);
	}
	void EssentialMatrixChanged(sofa::core::objectmodel::BaseObject*)
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

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_STEREOSETTINGS_H
