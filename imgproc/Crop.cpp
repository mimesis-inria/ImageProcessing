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

#include "Crop.h"

namespace sofacv
{
namespace imgproc
{

Crop::Crop()
    : d_roi(initData(&d_roi, "ROI", "x, y, w, h values of the ROI"))
{
}

void Crop::applyFilter(const cv::Mat &in,
                                                   cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: Resize requires a source and dest image";
    return;
  }
  cv::Rect roi;
  if (!d_roi.isSet())
    roi = cv::Rect(0, 0, in.cols, in.rows);
  else
    roi = cv::Rect(d_roi.getValue()[0], d_roi.getValue()[1],
                   d_roi.getValue()[2], d_roi.getValue()[3]);

  out = in(roi).clone();
}

void Crop::init()
{
  registerData(&d_roi, 0, 2000, 1);
  ImageFilter::init();
}

SOFA_DECL_CLASS(Crop)

int CropClass =
    sofa::core::RegisterObject("OpenCV's Crop function").add<Crop>();

}  // namespace imgproc
}  // namespace sofacv

