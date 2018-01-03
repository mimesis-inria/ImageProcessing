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

#include "AddWeighted.h"

sofaor::processor::imgproc::AddWeighted::AddWeighted()
    : d_img2(initData(&d_img2, "img2", "Image to add to img"))
{
}

void sofaor::processor::imgproc::AddWeighted::init() { ImageFilter::init(); }

void sofaor::processor::imgproc::AddWeighted::applyFilter(const cv::Mat &in,
                                                          cv::Mat &out, bool)
{
  if (in.empty() || d_img2.getValue().empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: cv::add requires a source and dest image";
    return;
  }

  cv::add(in, d_img2.getValue(), out);
}
