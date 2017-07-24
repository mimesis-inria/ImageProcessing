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

#ifndef SOFA_OR_PROCESSOR_CVTCOLOR_H
#define SOFA_OR_PROCESSOR_CVTCOLOR_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
/**
 * @brief Converts an image from one color space to another.
 *
 * Please refer to OpenCV's ColorConversionCodes enumeration in imgproc.hpp for
 * color codes
 */
class CvtColor : public ImageFilter
{
 public:
  SOFA_CLASS(CvtColor, ImageFilter);

  sofa::Data<double> d_code;
  sofa::Data<double> d_dstCn;

  CvtColor()
      : d_code(initData(&d_code, "code", "color space conversion code")),
        d_dstCn(initData(&d_dstCn, 0, "dstCn",
                         "[OPTIONAL] number of channels in the destination "
                         "image; if the parameter is 0, the number of the "
                         "channels is derived automatically from src and code"))
  {
  }

  void init()
  {
    if (!d_code.isSet())
      msg_error(getName() + "::init()") << "conversion code not provided";

    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    try
    {
      cv::cvtColor(in, out, d_code.getValue(), d_dstCn.getValue());
    }
    catch (cv::Exception& e)
    {
      msg_error(getName() + "::applyFilter()")
          << "Exception thrown by cv::cvtColor()" << e.what();
      out = in;
    }
  }
};

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CVTCOLOR_H
