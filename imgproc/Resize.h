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

#ifndef SOFA_OR_PROCESSOR_RESIZE_H
#define SOFA_OR_PROCESSOR_RESIZE_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class Resize : public ImageFilter
{
 public:
  SOFA_CLASS(Resize, ImageFilter);

  sofa::Data<sofa::defaulttype::Vec2i> d_size;
  sofa::Data<double> d_fx;
  sofa::Data<double> d_fy;
  sofa::Data<sofa::helper::OptionsGroup> d_interp;

  Resize()
      : d_size(
            initData(&d_size, "size", "pixel resolution of the output image")),
        d_fx(initData(&d_fx, "fx", "scale factor on the X axis")),
        d_fy(initData(&d_fy, "fy", "scale factori on the Y axis")),
        d_interp(initData(&d_interp, "interpolation", "interpolation method"))
  {
    sofa::helper::OptionsGroup* opt = d_interp.beginEdit();
    opt->setNames(5, "NEAREST", "LINEAR", "CUBIC", "AREA", "LANCZOS4");
    opt->setSelectedItem(1);
    d_interp.endEdit();
  }

  void init() { ImageFilter::init(); }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty())
    {
      msg_error(getName() + "::applyFilter()")
          << "Error: Resize requires a source and dest image";
      return;
    }
    cv::Size s;

    if (d_size.isSet())
    {
      s.width = d_size.getValue().x();
      s.height = d_size.getValue().y();
    }
    cv::resize(in, out, s, d_fx.getValue(), d_fy.getValue(),
               d_interp.getValue().getSelectedId());
  }
};

SOFA_DECL_CLASS(Resize)

int ResizeClass =
    sofa::core::RegisterObject("OpenCV's Resize function").add<Resize>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_RESIZE_H
