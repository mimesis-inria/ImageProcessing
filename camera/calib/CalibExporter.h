#ifndef SOFA_OR_PROCESSOR_CALIBEXPORTER_H
#define SOFA_OR_PROCESSOR_CALIBEXPORTER_H

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

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
class CalibExporter : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibExporter, StereoSettings,
																							sofa::BaseLink::FLAG_STOREPATH |
																									sofa::BaseLink::FLAG_STRONGLINK>
			StereoCam;

	typedef sofa::core::objectmodel::SingleLink<CalibExporter, CameraSettings,
																							sofa::BaseLink::FLAG_STOREPATH |
																									sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(CalibExporter, common::ImplicitDataEngine);

	CalibExporter();
	virtual ~CalibExporter();

	void init();
	void update();
	void cleanup();

	StereoCam l_sCam;
	CamSettings l_cam1;
	CamSettings l_cam2;
	sofa::core::objectmodel::DataFileName d_calibFolder;
	sofa::Data<std::string> d_calibName;
	sofa::Data<unsigned> d_nSteps;
	sofa::Data<sofa::helper::OptionsGroup> d_exportType;
	sofa::Data<bool> d_activate;

 protected:
	void calibFolderChanged(sofa::core::objectmodel::BaseObject*);
	unsigned m_stepCounter;
	bool m_isStereo;

 private:
	void exportCalib(const std::string& calibFile);
	bool canExport(const std::string& calibDir,
								 const std::string& calibFile) const;
	void export_cam(cv::Mat K, cv::Mat T, cv::Mat R, cv::FileStorage fs, double e, cv::Mat dv, cv::Mat res);
};

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CALIBEXPORTER_H
