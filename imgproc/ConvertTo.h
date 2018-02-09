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

#ifndef SOFA_OR_PROCESSOR_CONVERTTO_H
#define SOFA_OR_PROCESSOR_CONVERTTO_H

#include "common/ImageFilter.h"

#define SHOWVAL(v) #v

namespace sofaor
{
namespace processor
{
namespace imgproc
{
/**
 * @brief The ConvertTo class
 *
 * Converts a cvMat to another with different data type and optional scaling of
 * the values. (equivalent of the cv::Mat::convertTo() method) This is used to,
 * for instance, convert a CV_32F to a CV_8U by scaling every values from
 * 0 < val < 1 to 0 < val < 256 for instance.
 */
template <class T>
class ConvertTo : public ImageFilter
{
  T internal_type;

 public:
  SOFA_CLASS(ConvertTo, ImageFilter);

  sofa::Data<double> d_alpha;
  sofa::Data<double> d_beta;

  ConvertTo();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

  int getCVType(T type);
};

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_CONVERTTO_H
