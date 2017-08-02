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

namespace sofaor
{
namespace processor
{
namespace cam
{
/**
 * \brief Code related to mono / stereo Camera calibration, and pose estimation
 */
namespace calib
{
class CalibLoader : public common::ImplicitDataEngine
{
  SOFAOR_CALLBACK_SYSTEM(CalibLoader);
  typedef sofa::core::objectmodel::SingleLink<
      CalibLoader, StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      StereoCam;

  typedef sofa::core::objectmodel::SingleLink<
      CalibLoader, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

  struct CalibData
  {
    CalibData() {}
    CalibData(const sofa::defaulttype::Matrix3& _K1,
              const sofa::defaulttype::Matrix3& _R1,
              const sofa::defaulttype::Vector3& _T1,
              const sofa::helper::vector<double>& _delta1,
              const sofa::defaulttype::Vec2i _imSize1, double _error1,
              const sofa::defaulttype::Matrix3& _K2,
              const sofa::defaulttype::Matrix3& _R2,
              const sofa::defaulttype::Vector3& _T2,
              const sofa::helper::vector<double>& _delta2,
              const sofa::defaulttype::Vec2i _imSize2, double _error2,
              const sofa::defaulttype::Matrix3& _Rs,
              const sofa::defaulttype::Vector3& _Ts,
              const sofa::defaulttype::Matrix3& _F,
              const sofa::defaulttype::Matrix3& _E, double _totalError)
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

    sofa::defaulttype::Vec2i imSize1;
    sofa::defaulttype::Matrix3 K1;
    sofa::defaulttype::Matrix3 R1;
    sofa::defaulttype::Vector3 T1;
    sofa::helper::vector<double> delta1;
    double error1;

    sofa::defaulttype::Vec2i imSize2;
    sofa::defaulttype::Matrix3 K2;
    sofa::defaulttype::Matrix3 R2;
    sofa::defaulttype::Vector3 T2;
    sofa::helper::vector<double> delta2;
    double error2;

    sofa::defaulttype::Matrix3 Rs;
    sofa::defaulttype::Vector3 Ts;
    sofa::defaulttype::Matrix3 F;
    sofa::defaulttype::Matrix3 E;
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
  sofa::Data<sofa::helper::OptionsGroup> d_calibNames;
  bool m_isStereo;

  sofa::Data<sofa::defaulttype::Vec2i> d_imSize1;
  sofa::Data<sofa::defaulttype::Matrix3> d_K1;
  sofa::Data<sofa::defaulttype::Matrix3> d_R1;
  sofa::Data<sofa::defaulttype::Vector3> d_T1;
  sofa::Data<sofa::helper::vector<double> > d_delta1;
  sofa::Data<double> d_error1;

  sofa::Data<sofa::defaulttype::Vec2i> d_imSize2;
  sofa::Data<sofa::defaulttype::Matrix3> d_K2;
  sofa::Data<sofa::defaulttype::Matrix3> d_R2;
  sofa::Data<sofa::defaulttype::Vector3> d_T2;
  sofa::Data<sofa::helper::vector<double> > d_delta2;
  sofa::Data<double> d_error2;

  sofa::Data<sofa::defaulttype::Matrix3> d_Rs;
  sofa::Data<sofa::defaulttype::Vector3> d_Ts;
  sofa::Data<sofa::defaulttype::Matrix3> d_F;
  sofa::Data<sofa::defaulttype::Matrix3> d_E;
  sofa::Data<double> d_totalError;

 protected:
  std::map<std::string, CalibData> m_calibs;

  void calibChanged(sofa::core::objectmodel::BaseData*);
  void calibFolderChanged(sofa::core::objectmodel::BaseData*);

 private:
  void load(const std::string& calibfile);
  bool canLoad(const std::string& calibfile) const;
  void setCurrentCalib(CalibData& d);
  void setCurrentCalib(const std::string& calibName);
  std::string getPathToCalibs();
  void getAllCalibFiles(std::vector<std::string>& calibFiles);
};

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CALIBLOADER_H
