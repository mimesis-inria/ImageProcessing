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

#ifndef SOFA_OR_PROCESSOR_FILL_H
#define SOFA_OR_PROCESSOR_FILL_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class Fill : public ImageFilter
{
  SOFAOR_CALLBACK_SYSTEM(Fill);

 public:
  SOFA_CLASS(Fill, ImageFilter);

  sofa::Data<sofa::defaulttype::Vec4d> d_color;

  Fill()
      : d_color(initData(&d_color, sofa::defaulttype::Vec4d(1.0, 1.0, 1.0, 1.0),
                         "scalar", "pixel color value."))
  {
  }

  void init()
  {
    registerData(&d_color, 0.0, 1.0, 0.0001);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;

    sofa::defaulttype::Vec4d color;
    if (in.depth() == CV_8U)
    {
      color = d_color.getValue() * 255;
    }
    else if (in.depth() == CV_32F)
    {
      color = d_color.getValue();
    }
    try
    {
      in.copyTo(out);
      out.setTo(cv::Scalar(color.x(), color.y(), color.z(), color.w()));
    }
    catch (cv::Exception& e)
    {
      std::cout << e.what() << std::endl;
      return;
    }
  }
};

SOFA_DECL_CLASS(Fill)

int FillClass =
    sofa::core::RegisterObject("OpenCV's implementation of cv::Mat::setTo()")
        .add<Fill>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FILL_H
