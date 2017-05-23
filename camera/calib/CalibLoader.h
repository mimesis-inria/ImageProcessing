#ifndef SOFA_OR_PROCESSOR_CALIBLOADER_H
#define SOFA_OR_PROCESSOR_CALIBLOADER_H

#include "camera/common/CameraSettings.h"
#include "camera/common/StereoSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMat.h>
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/core/core.hpp>

#include <map>

namespace sofa
{
namespace OR
{
namespace processor
{
class CalibLoader : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibLoader, StereoSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			StereoCam;

	typedef sofa::core::objectmodel::SingleLink<CalibLoader, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

  struct CalibData
  {
    CalibData() {}
		CalibData(const defaulttype::Matrix3& _K1, const defaulttype::Matrix3& _R1,
							const defaulttype::Vector3& _T1,
							const helper::vector<double>& _delta1,
							const defaulttype::Vec2i _imSize1, double _error1,
							const defaulttype::Matrix3& _K2, const defaulttype::Matrix3& _R2,
							const defaulttype::Vector3& _T2,
							const helper::vector<double>& _delta2,
							const defaulttype::Vec2i _imSize2, double _error2,
							const defaulttype::Matrix3& _Rs, const defaulttype::Vector3& _Ts,
							const defaulttype::Matrix3& _F, const defaulttype::Matrix3& _E,
							double _totalError)
				: imSize1(_imSize1),
					K1(_K1),
					R1(_R1),
					T1(_T1),
					delta1(_delta1),
          error1(_error1),
					imSize2(_imSize2),
					K2(_K2),
					R2(_R2),
					T2(_T2),
					delta2(_delta2),
					error2(_error2),
					Rs(_Rs),
					Ts(_Ts),
          F(_F),
					E(_E),
					totalError(_totalError)
    {
    }

		defaulttype::Vec2i imSize1;
		defaulttype::Matrix3 K1;
		defaulttype::Matrix3 R1;
		defaulttype::Vector3 T1;
		helper::vector<double> delta1;
		double error1;

		defaulttype::Vec2i imSize2;
		defaulttype::Matrix3 K2;
		defaulttype::Matrix3 R2;
		defaulttype::Vector3 T2;
		helper::vector<double> delta2;
		double error2;

		defaulttype::Matrix3 Rs;
		defaulttype::Vector3 Ts;
    defaulttype::Matrix3 F;
		defaulttype::Matrix3 E;
		double totalError;
  };

 public:
  SOFA_CLASS(CalibLoader, common::ImplicitDataEngine);

  CalibLoader();
  virtual ~CalibLoader();

  void init();
  void update();

	StereoCam l_sCam;
	CamSettings l_cam1;
	CamSettings l_cam2;
	sofa::core::objectmodel::DataFileName d_calibFolder;
  Data<helper::OptionsGroup> d_calibNames;
  Data<bool> d_isStereo;

	Data<defaulttype::Vec2i> d_imSize1;
	Data<defaulttype::Matrix3> d_K1;
	Data<defaulttype::Matrix3> d_R1;
	Data<defaulttype::Vector3> d_T1;
	Data<helper::vector<double> > d_delta1;
	Data<double> d_error1;

	Data<defaulttype::Vec2i> d_imSize2;
	Data<defaulttype::Matrix3> d_K2;
	Data<defaulttype::Matrix3> d_R2;
	Data<defaulttype::Vector3> d_T2;
	Data<helper::vector<double> > d_delta2;
	Data<double> d_error2;

	Data<defaulttype::Matrix3> d_Rs;
	Data<defaulttype::Vector3> d_Ts;
  Data<defaulttype::Matrix3> d_F;
	Data<defaulttype::Matrix3> d_E;
	Data<double> d_totalError;

 protected:
  std::map<std::string, CalibData> m_calibs;

  void calibChanged(core::objectmodel::BaseObject*);
  void calibFolderChanged(core::objectmodel::BaseObject*);

 private:
  void load(const std::string& calibfile);
  bool canLoad(const std::string& calibfile) const;
  void setCurrentCalib(CalibData& d);
  void setCurrentCalib(const std::string& calibName);
  std::string getPathToCalibs();
  void getAllCalibFiles(std::vector<std::string>& calibFiles);
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CALIBLOADER_H
