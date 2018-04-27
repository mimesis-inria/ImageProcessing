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

#ifndef SOFACV_CAM_CALIB_CALIBRATESTEREO_H
#define SOFACV_CAM_CALIB_CALIBRATESTEREO_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include "camera/common/StereoSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
namespace calib
{
class SOFA_IMAGEPROCESSING_API CalibrateStereo : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      CalibrateStereo, StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      Settings;

 public:
  SOFA_CLASS(CalibrateStereo, ImplicitDataEngine);

  CalibrateStereo();

  ~CalibrateStereo() {}
  void init() override;

  virtual void Update() override;
  void calibrate();

  Settings l_cam;

  // INPUTS
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector2> > >
      d_imagePoints1;
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector2> > >
      d_imagePoints2;
  sofa::Data<sofa::helper::SVector<
      sofa::helper::SVector<sofa::defaulttype::Vector3> > >
      d_objectPoints;
  sofa::Data<sofa::defaulttype::Vec2i> d_imgSize;

  // OPTIONAL INPUTS
  sofa::Data<int> d_calibFlags;
};

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CALIB_CALIBRATESTEREO_H
