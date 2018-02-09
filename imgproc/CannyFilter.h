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

#ifndef SOFA_OR_PROCESSOR_CANNYFILTER_H
#define SOFA_OR_PROCESSOR_CANNYFILTER_H

#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class CannyFilter : public ImageFilter
{
 public:
  SOFA_CLASS(CannyFilter, ImageFilter);

  sofa::Data<double> d_minThreshold;
  sofa::Data<double> d_maxThreshold;
  sofa::Data<int> d_apertureSize;
  sofa::Data<bool> d_l2gradient;

  CannyFilter();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);
};

SOFA_DECL_CLASS(CannyFilter)

int CannyFilterClass =
    sofa::core::RegisterObject("Canny edge detection filter from OpenCV")
        .add<CannyFilter>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CANNYFILTER_H
