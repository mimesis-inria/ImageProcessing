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

#include "ConvertTo.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofaor
{
namespace processor
{
namespace imgproc
{
SOFA_DECL_CLASS(ConvertTo)

int ConvertToClass = sofa::core::RegisterObject(
                         "Converts OpenCV Matrices types, and optionally "
                         "scales / crop the histogram using alpha & beta")
                         .add<ConvertTo<char> >()
                         .add<ConvertTo<unsigned char> >()
                         .add<ConvertTo<short> >()
                         .add<ConvertTo<unsigned short> >()
                         .add<ConvertTo<int> >()
                         .add<ConvertTo<float> >()
                         .add<ConvertTo<double> >();

template <class T>
ConvertTo::ConvertTo()
    : d_alpha(initData(&d_alpha, "scale", "[OPTIONAL] scale factor")),
      d_beta(initData(&d_beta, "delta",
                      "[OPTIONAL] delta added to the scaled values"))
{
}

template <class T>
void ConvertTo::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty()) return;
  in.convertTo(out, getCVType(internal_type), d_alpha.getValue(),
               d_beta.getValue());
}

template <class T>
void ConvertTo::init()
{
  ImageFilter::init();
}

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
