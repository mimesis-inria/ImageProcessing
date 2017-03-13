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
 public:
	SOFA_CLASS(SOFA_TEMPLATE2(PointVectorConverter, SrcType, DstType),
						 common::ImplicitDataEngine);

	PointVectorConverter()
			: d_src(initData(&d_src, "points", "input vector to convert")),
				d_dst(initData(&d_dst, "points_out", "converted output vector"))
	{
	}

	~PointVectorConverter() {}
	void init()
	{
		addInput(&d_src);
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
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
