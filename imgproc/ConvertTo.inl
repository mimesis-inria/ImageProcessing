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

#include "ConvertTo.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{

template <class T>
ConvertTo<T>::ConvertTo()
    : d_alpha(initData(&d_alpha, 1.0, "scale", "scale factor (default 1.0)")),
      d_beta(initData(&d_beta, 0.0, "delta",
                      "[OPTIONAL] delta added to the scaled values (default 0.0)"))
{
}

template <class T>
void ConvertTo<T>::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty()) return;
  in.convertTo(out, getCVType(internal_type), d_alpha.getValue(),
               d_beta.getValue());
}

template <class T>
void ConvertTo<T>::init()
{
  ImageFilter::init();
}

template <class T>
int ConvertTo<T>::getCVType(T)
{
  return 0;
}

template <>
int ConvertTo<char>::getCVType(char)
{
  return CV_8S;
}
template <>
int ConvertTo<unsigned char>::getCVType(unsigned char)
{
  return CV_8U;
}
template <>
int ConvertTo<short>::getCVType(short)
{
  return CV_16S;
}
template <>
int ConvertTo<unsigned short>::getCVType(unsigned short)
{
  return CV_16U;
}

template <>
int ConvertTo<int>::getCVType(int)
{
  return CV_32S;
}

template <>
int ConvertTo<float>::getCVType(float)
{
  return CV_32F;
}

template <>
int ConvertTo<double>::getCVType(double)
{
  return CV_64F;
}

template <class T>
std::string ConvertTo<T>::templateName(
       const ConvertTo<T>* /*ptr*/)
{
    return std::string(sofa::defaulttype::DataTypeName<T>::name());
}


}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
