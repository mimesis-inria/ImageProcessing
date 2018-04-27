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

#include "NegateVector.h"

namespace sofacv
{
namespace utils
{

void NegateVector::init()
{
  addInput(&d_src);
//    addInput(&d_srcRot);
  addOutput(&d_dst);
}

void NegateVector::Update()
{
  d_dst.setValue(-d_src.getValue());
//    sofa::defaulttype::Matrix3 rev;
//    d_dstRot.setValue(-d_srcRot.getValue().transposed());
//    d_dst.setValue(d_dstRot.getValue() * d_dst.getValue());
}


SOFA_DECL_CLASS(NegateVector)

int NegateVectorClass =
    sofa::core::RegisterObject("component to negate a defaulttype::Vector3")
        .add<NegateVector>();

}  // namespace utils
}  // namespace sofacv
