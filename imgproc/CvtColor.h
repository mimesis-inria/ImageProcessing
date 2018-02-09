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

  sofa::Data<int> d_code;
  sofa::Data<int> d_dstCn;

  CvtColor();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);
};

SOFA_DECL_CLASS(CvtColor)

int CvtColorClass = sofa::core::RegisterObject(
                        "Converts an image from one color space to another.")
                        .add<CvtColor>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CVTCOLOR_H
