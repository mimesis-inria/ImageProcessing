#ifndef SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
#define SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H

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
	//	typedef sofa::core::objectmodel::SingleLink<PointVectorConverter,
	// core::objectmodel::BaseObject,
	//																							BaseLink::FLAG_STOREPATH
	//|
	//																									BaseLink::FLAG_STRONGLINK>
	//			LinkFunctor;

 public:
	SOFA_CLASS(SOFA_TEMPLATE2(PointVectorConverter, SrcType, DstType),
						 common::ImplicitDataEngine);

	PointVectorConverter()
			: d_src(initData(&d_src, "points", "input vector to convert")),
				d_dst(initData(&d_dst, "points_out", "converted output vector")),
				d_projection(initData(
						&d_projection, "P",
						"projection matrix (retrieved from CameraSettings for instance")),
						d_depth(initData(
								&d_depth, "depth",
								"depth at which you want to see your image's points projected"))
	//				l_functor(
	//						initLink("operation", "link to an optional
	//functor
	// component"))
	{
	}

	~PointVectorConverter() {}
	void init()
	{
		addInput(&d_src);
		addInput(&d_projection);
		addInput(&d_depth);

		addOutput(&d_dst);
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

	Data<defaulttype::Mat3x4d> d_projection;
	Data<double> d_depth;

	//	LinkFunctor l_functor;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
