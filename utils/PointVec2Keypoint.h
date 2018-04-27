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

#ifndef SOFACV_UTILS_POINTVEC2KEYPOINT_H
#define SOFACV_UTILS_POINTVEC2KEYPOINT_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace utils
{
class SOFA_IMAGEPROCESSING_API PointVec2Keypoint : public ImplicitDataEngine
{
 public:
  SOFA_CLASS(PointVec2Keypoint, ImplicitDataEngine);

  PointVec2Keypoint()
      : d_src(initData(&d_src, "points", "input vector to convert")),
        d_dst(initData(&d_dst, "points_out", "converted output vector"))
  {
  }

  virtual ~PointVec2Keypoint() override {}
  void init() override;
  void Update() override;

  // INPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_src;
  // OUTPUTS
    sofa::Data<sofa::helper::vector<cvKeypoint> > d_dst;
};


}  // namespace utils
}  // namespace sofacv

#endif  // SOFACV_UTILS_POINTVEC2KEYPOINT_H
