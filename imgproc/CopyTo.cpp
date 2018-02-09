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

#include "CopyTo.h"

sofaor::processor::imgproc::CopyTo::CopyTo()
    : d_mask(initData(&d_mask, "mask", "mask")),
      d_useMask(initData(&d_useMask, true, "useMask",
                         "whether or not to use the input mask"))
{
}

void sofaor::processor::imgproc::CopyTo::applyFilter(const cv::Mat &in,
                                                     cv::Mat &out, bool)
{
  if (in.empty())
  {
    msg_error(getName() + "::applyFilter()")
        << "Error: copyTo requires a source and dest image";
    return;
  }

  in.copyTo(out,
            (d_useMask.getValue()) ? (d_mask.getValue()) : (common::cvMat()));
}

void sofaor::processor::imgproc::CopyTo::init()
{
  registerData(&d_useMask);
  ImageFilter::init();
}
