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

#include "PointVec2Keypoint.h"

namespace sofacv
{
namespace utils
{

void PointVec2Keypoint::init()
{
  addInput(&d_src);
  addOutput(&d_dst);
}

void PointVec2Keypoint::Update()
{
      sofa::helper::vector<cvKeypoint>& dst = *(d_dst.beginWriteOnly());
  dst.clear();
      const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();
  for (auto pt : src)
    dst.push_back(cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}


SOFA_DECL_CLASS(PointVec2Keypoint)

int PointVec2KeypointClass =
        sofa::core::RegisterObject(
        "component to convert defaulttype::vec2i to common::cvKeypoint")
        .add<PointVec2Keypoint>();
}  // namespace utils
}  // namespace sofacv
