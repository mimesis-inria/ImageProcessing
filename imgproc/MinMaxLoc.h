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

#ifndef SOFA_OR_PROCESSOR_MINMAXLOC_H
#define SOFA_OR_PROCESSOR_MINMAXLOC_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class MinMaxLoc : public ImageFilter
{
  SOFAOR_CALLBACK_SYSTEM(MinMaxLoc);

 public:
  SOFA_CLASS(MinMaxLoc, ImageFilter);

  sofa::Data<double> d_min;
  sofa::Data<double> d_max;
  sofa::Data<sofa::defaulttype::Vec2i> d_minLoc;
  sofa::Data<sofa::defaulttype::Vec2i> d_maxLoc;
  sofa::Data<common::cvMat> d_mask;

  MinMaxLoc()
      : d_min(initData(&d_min, "min", "min score", true, true)),
        d_max(initData(&d_max, "max", "max score", true, true)),
        d_minLoc(initData(&d_minLoc, "minLoc",
                          "pointer to the min score's location", true, true)),
        d_maxLoc(initData(&d_maxLoc, "maxLoc",
                          "pointer to the max score's location", true, true)),
        d_mask(initData(&d_mask, "mask", "image mask to reduce searching area"))
  {
  }

  void init() { ImageFilter::init(); }

  void applyFilter(const cv::Mat& in, cv::Mat& /*out*/, bool)
  {
    if (in.empty()) return;

    if (in.channels() != 1)
    {
      msg_error("MinMaxLoc::applyFilter()")
          << "MinMaxLoc takes single channelled images";
    }
    try
    {
      double min, max;
      cv::Point minl, maxl;
      cv::minMaxLoc(in, &min, &max, &minl, &maxl,
                    (d_mask.isSet()) ? (d_mask.getValue()) : (cv::noArray()));
      d_min.setValue(min);
      d_max.setValue(max);
      d_minLoc.setValue(sofa::defaulttype::Vec2i(minl.x, minl.y));
      d_maxLoc.setValue(sofa::defaulttype::Vec2i(maxl.x, maxl.y));
    }
    catch (cv::Exception& e)
    {
      std::cout << e.what() << std::endl;
      return;
    }
  }
};

SOFA_DECL_CLASS(MinMaxLoc)

int MinMaxLocClass =
    sofa::core::RegisterObject("OpenCV's implementation of cv::MinMaxLoc")
        .add<MinMaxLoc>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_MINMAXLOC_H
