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

#ifndef SOFA_OR_PROCESSOR_CALIBRATECAMERA_H
#define SOFA_OR_PROCESSOR_CALIBRATECAMERA_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "camera/common/CameraSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
class CalibrateCamera : public common::ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CalibrateCamera, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(CalibrateCamera, common::ImplicitDataEngine);

  CalibrateCamera();

  ~CalibrateCamera() {}
  void init();

  virtual void Update() override;
  void calibrate();

  CamSettings l_cam;

  // INPUTS
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector2> > >
      d_imagePoints;
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector3> > >
      d_objectPoints;
  sofa::Data<sofa::defaulttype::Vec2i> d_imgSize;

  // OPTIONAL INPUTS
  sofa::Data<int> d_calibFlags;
  sofa::Data<sofa::defaulttype::Matrix3> d_K;
  sofa::Data<sofa::helper::vector<double> > d_distCoefs;

  // OUTPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Mat3x4d> > d_Rts;

  sofa::Data<bool> d_preserveExtrinsics;

 private:
  sofa::defaulttype::Matrix3 m_K;
  sofa::helper::vector<double> m_distCoefs;
};

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CALIBRATECAMERA_H
