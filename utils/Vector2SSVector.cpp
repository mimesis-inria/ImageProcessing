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

#include "Vector2SSVector.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace utils
{
SOFA_DECL_CLASS(Vector2SSVector)

int Vector2SSVectorClass =
		sofa::core::RegisterObject("Converts vector to vectors of vectors")
				.add<Vector2SSVector<sofa::defaulttype::Vec2i> >()
                .add<Vector2SSVector<cvKeypoint> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec2f> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec3f> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec2d> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec3d> >();
}  // namespace utils
}  // namespace sofacv
