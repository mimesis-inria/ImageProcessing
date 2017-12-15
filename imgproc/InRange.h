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

#ifndef SOFA_OR_PROCESSOR_INRANGE_H
#define SOFA_OR_PROCESSOR_INRANGE_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class InRange : public ImageFilter
{
    typedef typename sofa::defaulttype::Vec3i Vec3i;
 public:
  SOFA_CLASS(InRange, ImageFilter);

  sofa::Data<Vec3i> d_minRange;
  sofa::Data<Vec3i> d_maxRange;

  InRange()
      : d_minRange(initData(&d_minRange, Vec3i(0,0,0), "minRange", "minimum color value")),
        d_maxRange(initData(&d_maxRange, Vec3i(255,255,255), "maxRange", "max color value"))
  {
  }

  void init()
  {
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty())
    {
      msg_error(getName() + "::applyFilter()")
          << "Error: cv::add requires a source and dest image";
      return;
    }

    cv::Scalar min(d_minRange.getValue().x(), d_minRange.getValue().y(), d_minRange.getValue().z());
    cv::Scalar max(d_maxRange.getValue().x(), d_maxRange.getValue().y(), d_maxRange.getValue().z());

    in.copyTo(out);
    cv::inRange(in, min, max, out);
  }
};

SOFA_DECL_CLASS(InRange)

int InRangeClass =
    sofa::core::RegisterObject("OpenCV's InRange function")
        .add<InRange>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_INRANGE_H
