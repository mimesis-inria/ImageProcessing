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

#ifndef SOFA_OR_PROCESSOR_SOBELFILTER_H
#define SOFA_OR_PROCESSOR_SOBELFILTER_H

#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class SobelFilter : public ImageFilter
{
 public:
  SOFA_CLASS(SobelFilter, ImageFilter);

  sofa::Data<int> d_ddepth;
  sofa::Data<int> d_xorder;
  sofa::Data<int> d_yorder;
  sofa::Data<int> d_ksize;
  sofa::Data<double> d_scale;
  sofa::Data<double> d_delta;
  sofa::Data<sofa::helper::OptionsGroup> d_bordertype;

  SobelFilter();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);
};

SOFA_DECL_CLASS(SobelFilter)

int SobelFilterClass =
    sofa::core::RegisterObject("OpenCV's implementation of the Sobel operator")
        .add<SobelFilter>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_SOBELFILTER_H
