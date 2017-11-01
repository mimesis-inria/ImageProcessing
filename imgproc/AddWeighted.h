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

#ifndef SOFA_OR_PROCESSOR_ADDWEIGHTED_H
#define SOFA_OR_PROCESSOR_ADDWEIGHTED_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class AddWeighted : public ImageFilter
{
 public:
  SOFA_CLASS(AddWeighted, ImageFilter);

  sofa::Data<common::cvMat> d_img2;

  AddWeighted()
      : d_img2(initData(&d_img2, "img2", "Image to add to img"))
  {
  }

  void init()
  {
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty() || d_img2.getValue().empty())
    {
      msg_error(getName() + "::applyFilter()")
          << "Error: cv::add requires a source and dest image";
      return;
    }

    cv::add(in, d_img2.getValue(), out);
  }
};

SOFA_DECL_CLASS(AddWeighted)

int AddWeightedClass =
    sofa::core::RegisterObject("OpenCV's AddWeighted function")
        .add<AddWeighted>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_ADDWEIGHTED_H
