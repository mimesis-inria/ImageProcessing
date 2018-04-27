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

#ifndef SOFACV_UTILS_NEGATEVECTOR_H
#define SOFACV_UTILS_NEGATEVECTOR_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include <sofa/defaulttype/Mat.h>

namespace sofacv
{
namespace utils
{
class SOFA_IMAGEPROCESSING_API NegateVector : public ImplicitDataEngine
{
 public:
  SOFA_CLASS(NegateVector, ImplicitDataEngine);

  NegateVector()
      : d_src(initData(&d_src, "input_position", "input vector to negate")),
//        d_srcRot(
//            initData(&d_srcRot, "input_rotation", "input rotation to negate")),
        d_dst(initData(&d_dst, "output_position", "negated output vector"))/*,*/
//        d_dstRot(
//            initData(&d_dstRot, "output_rotation", "negated output vector"))
  {
  }

  virtual ~NegateVector() override {}
  void init() override;
  void Update() override;
  // INPUTS
  sofa::Data<sofa::defaulttype::Vector3> d_src;
  sofa::Data<sofa::defaulttype::Matrix3> d_srcRot;
  // OUTPUTS
  sofa::Data<sofa::defaulttype::Vector3> d_dst;
//  sofa::Data<sofa::defaulttype::Matrix3> d_dstRot;
};

}  // namespace utils
}  // namespace sofacv

#endif  // SOFACV_UTILS_NEGATEVECTOR_H
