#ifndef SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
#define SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H

#include "calib/CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvKeypoint.h>

namespace sofa
{
namespace OR
{
namespace processor
{
template <class SrcType, class DstType>
class PointVectorConverter : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<
			PointVectorConverter, CameraSettings,
			BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK>
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

	void update();

	virtual std::string getTemplateName() const { return templateName(this); }
	static std::string templateName(
			const PointVectorConverter<SrcType, DstType>* = NULL);

	// INPUTS
	Data<helper::vector<SrcType> > d_src;
	// OUTPUTS
	Data<helper::vector<DstType> > d_dst;

	CamSettings l_cam;
	Data<double> d_depth;

	//	LinkFunctor l_functor;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
