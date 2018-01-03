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
  typedef sofa::core::objectmodel::SingleLink<
      CalibExporter, StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      StereoCam;

  typedef sofa::core::objectmodel::SingleLink<
      CalibExporter, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
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
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CALIBEXPORTER_H
