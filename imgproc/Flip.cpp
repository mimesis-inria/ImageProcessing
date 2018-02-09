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

#include "Flip.h"

sofaor::processor::imgproc::Flip::Flip()
    : d_flipCode(initData(&d_flipCode, 1, "flipCode",
                          "0 for X axis flip, 1 for Y, and -1 for both"))
{
}

void sofaor::processor::imgproc::Flip::init()
{
  registerData(&d_flipCode, -1, 1, 1);
  ImageFilter::init();
}

void sofaor::processor::imgproc::Flip::applyFilter(const cv::Mat &in,
                                                   cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: Flip requires a source and dest image";
    return;
  }

  cv::flip(in, out, d_flipCode.getValue());
}
