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

#ifndef SOFA_OR_PROCESSOR_VECTOR2SSVECTOR_H
#define SOFA_OR_PROCESSOR_VECTOR2SSVECTOR_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <sofa/helper/SVector.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
template <class T>
class Vector2SSVector : public common::ImplicitDataEngine
{
 public:
	SOFA_CLASS(SOFA_TEMPLATE(Vector2SSVector, T), common::ImplicitDataEngine);

	Vector2SSVector()
			: d_src(initData(&d_src, "srcType", "input vector to convert")),
				d_dst(initData(&d_dst, "dstType", "converted output vector"))
	{
	}

	~Vector2SSVector() {}
	void init()
	{
		addInput(&d_src);
		addOutput(&d_dst);
		update();
	}

  virtual void Update() override;

	virtual std::string getTemplateName() const { return templateName(this); }
	static std::string templateName(
			const Vector2SSVector<T>* = NULL);

	// INPUTS
	sofa::Data<sofa::helper::vector<T> > d_src;
	// OUTPUTS
	sofa::Data<sofa::helper::SVector<sofa::helper::SVector<T> > > d_dst;
};

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_VECTOR2SSVECTOR_H
