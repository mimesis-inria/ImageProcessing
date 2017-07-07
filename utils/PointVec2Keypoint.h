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

#ifndef SOFA_OR_PROCESSOR_POINTVEC2KEYPOINT_H
#define SOFA_OR_PROCESSOR_POINTVEC2KEYPOINT_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
class PointVec2Keypoint : public common::ImplicitDataEngine
{
 public:
  SOFA_CLASS(PointVec2Keypoint, common::ImplicitDataEngine);

  PointVec2Keypoint()
      : d_src(initData(&d_src, "points", "input vector to convert")),
        d_dst(initData(&d_dst, "points_out", "converted output vector"))
  {
  }

  ~PointVec2Keypoint() {}
  void init()
  {
    addInput(&d_src);
    addOutput(&d_dst);
  }

  void update()
  {
		sofa::helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
    dst.clear();
		const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();
    for (auto pt : src)
      dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
  }

  // INPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_src;
  // OUTPUTS
	sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_dst;
};

SOFA_DECL_CLASS(PointVec2Keypoint)

int PointVec2KeypointClass =
		sofa::core::RegisterObject(
        "component to convert defaulttype::vec2i to common::cvKeypoint")
        .add<PointVec2Keypoint>();

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_POINTVEC2KEYPOINT_H
