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

#include "CvtColor.h"

sofaor::processor::imgproc::CvtColor::CvtColor()
    : d_code(initData(&d_code, 6, "code",
                      "color space conversion code default is BGR2GRAY")),
      d_dstCn(initData(&d_dstCn, 0, "dstCn",
                       "[OPTIONAL] number of channels in the destination "
                       "image; if the parameter is 0, the number of the "
                       "channels is derived automatically from src and code"))
{
}

void sofaor::processor::imgproc::CvtColor::init() { ImageFilter::init(); }

void sofaor::processor::imgproc::CvtColor::applyFilter(const cv::Mat &in,
                                                       cv::Mat &out, bool)
{
  if (in.empty()) return;
  try
  {
    cv::cvtColor(in, out, d_code.getValue(), d_dstCn.getValue());
  }
  catch (cv::Exception &e)
  {
    msg_error(getName() + "::applyFilter()")
        << "Exception thrown by cv::cvtColor()" << e.what();
    out = in;
  }
}
