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

#ifndef SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
#define SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvKeypoint.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
template <class SrcType, class DstType>
class PointVectorConverter : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<
			PointVectorConverter, cam::CameraSettings,
			sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(SOFA_TEMPLATE2(PointVectorConverter, SrcType, DstType),
						 common::ImplicitDataEngine);

	PointVectorConverter()
			: l_cam(initLink("cam",
											 "link to CameraSettings component to reproject 2D "
											 "points in 3D n vice versa")),
				d_src(initData(&d_src, "points", "input vector to convert")),
				d_dst(initData(&d_dst, "points_out", "converted output vector")),
				d_depth(initData(
						&d_depth, "depth",
						"depth at which you want to see your image's points projected"))
	{
	}

	~PointVectorConverter() {}
	void init()
	{
		addInput(&d_src);
		addInput(&d_depth);

		addOutput(&d_dst);

		if (!l_cam.get())
			msg_warning(getName() + "::init()") << "Warning: No camera link set. "
																					 "Please use attribute 'cam' "
																					 "to define one";

		update();
	}

  virtual void Update() override;

	virtual std::string getTemplateName() const { return templateName(this); }
	static std::string templateName(
			const PointVectorConverter<SrcType, DstType>* = NULL);

	CamSettings l_cam;

	// INPUTS
	sofa::Data<sofa::helper::vector<SrcType> > d_src;
	// OUTPUTS
	sofa::Data<sofa::helper::vector<DstType> > d_dst;

	sofa::Data<double> d_depth;
};

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
