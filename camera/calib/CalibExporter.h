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

namespace sofa
{
namespace OR
{
namespace processor
{
class CalibExporter : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibExporter, StereoSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			StereoCam;

	typedef sofa::core::objectmodel::SingleLink<CalibExporter, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
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
	Data<std::string> d_calibName;
	Data<unsigned> d_nSteps;
	Data<helper::OptionsGroup> d_exportType;
	Data<bool> d_activate;

 protected:
	void calibFolderChanged(core::objectmodel::BaseObject*);
	unsigned m_stepCounter;
	bool m_isStereo;

 private:
	void exportCalib(const std::string& calibFile);
	bool canExport(const std::string& calibDir,
								 const std::string& calibFile) const;
	void export_cam(cv::Mat K, cv::Mat T, cv::Mat R, cv::FileStorage fs, double e, cv::Mat dv, cv::Mat res);
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_CALIBEXPORTER_H
