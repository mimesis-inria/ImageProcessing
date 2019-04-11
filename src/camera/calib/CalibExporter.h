#ifndef SOFACV_CAM_CALIB_CALIBEXPORTER_H
#define SOFACV_CAM_CALIB_CALIBEXPORTER_H

#include "ImageProcessingPlugin.h"

#include "camera/common/CameraSettings.h"
#include "camera/common/StereoSettings.h"

#include <SofaCV/SofaCV.h>

#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/core/core.hpp>

#include <map>

namespace sofacv
{
namespace cam
{
namespace calib
{
class SOFA_IMAGEPROCESSING_API CalibExporter : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CalibExporter, StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      StereoCam;

  typedef sofa::core::objectmodel::SingleLink<
      CalibExporter, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(CalibExporter, ImplicitDataEngine);

  CalibExporter();
  virtual ~CalibExporter();

  virtual void init() override;
  virtual void doUpdate() override;
  virtual void cleanup() override;

  StereoCam l_sCam;
  CamSettings l_cam1;
  CamSettings l_cam2;
  sofa::core::objectmodel::DataFileName d_calibFolder;
  sofa::Data<std::string> d_calibName;
  sofa::Data<unsigned> d_nSteps;
  sofa::Data<sofa::helper::OptionsGroup> d_exportType;
  sofa::Data<bool> d_activate;

 protected:
  void calibFolderChanged(sofa::core::objectmodel::BaseData*);
  unsigned m_stepCounter;
  bool m_isStereo;

 private:
  void exportCalib(const std::string& calibFile);
  bool canExport(const std::string& calibDir,
                 const std::string& calibFile) const;
  void export_cam(cv::Mat K, cv::Mat T, cv::Mat R, cv::FileStorage fs, double e,
                  cv::Mat dv, cv::Mat res);
};

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CALIB_CALIBEXPORTER_H
