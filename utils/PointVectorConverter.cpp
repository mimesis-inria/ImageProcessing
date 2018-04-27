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

#include "PointVectorConverter.inl"

#include <SofaCV/SofaCV.h>

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace utils
{
SOFA_DECL_CLASS(PointVectorConverter)

int PointVectorConverterClass =
		sofa::core::RegisterObject(
				"Converts vector of cvKeyPoints to sofa vectors, and vice versa")
                .add<PointVectorConverter<sofa::defaulttype::Vec2i, cvKeypoint> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec2d> >()
                .add<PointVectorConverter<cvKeypoint, sofa::defaulttype::Vec2i> >()
                .add<PointVectorConverter<sofa::defaulttype::Vec2d, cvKeypoint> >()
                .add<PointVectorConverter<cvKeypoint, sofa::defaulttype::Vec2d> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2d, sofa::defaulttype::Vec3d> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec3d> >();
}  // namespace utils
}  // namespace sofacv
