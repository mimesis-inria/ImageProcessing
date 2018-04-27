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

#include "Resize.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace sofacv
{
namespace imgproc
{

Resize::Resize()
    : d_size(initData(&d_size, "size", "pixel resolution of the output image")),
      d_fx(initData(&d_fx, 0.0, "fx", "scale factor on the X axis")),
      d_fy(initData(&d_fy, 0.0, "fy", "scale factori on the Y axis")),
      d_interp(initData(&d_interp, "interpolation", "interpolation method"))
{
  sofa::helper::OptionsGroup *opt = d_interp.beginEdit();
  opt->setNames(5, "NEAREST", "LINEAR", "CUBIC", "AREA", "LANCZOS4");
  opt->setSelectedItem(1);
  d_interp.endEdit();
}

void Resize::init()
{
  registerData(&d_interp);
  ImageFilter::init();
}

void Resize::applyFilter(const cv::Mat &in,
                                                     cv::Mat &out, bool)
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
             int(d_interp.getValue().getSelectedId()));
}

SOFA_DECL_CLASS(Resize)

int ResizeClass =
    sofa::core::RegisterObject("OpenCV's Resize function").add<Resize>();


} // namespace imgproc
} // namespace sofacv
