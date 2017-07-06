#ifndef SOFA_OR_PROCESSOR_CONVERSIONFUNCTOR_H
#define SOFA_OR_PROCESSOR_CONVERSIONFUNCTOR_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofaor
{
namespace processor
{
namespace utils
{

class ConversionFunctor : core::objectmodel::BaseObject
{
 public:
	SOFA_CLASS(ConversionFunctor, core::objectmodel::BaseObject);

	ConversionFunctor() {}
	~ConversionFunctor() {}

	template <class SrcType, class DstType>
	void convert(const helper::vector<SrcType>& src, helper::vector<DstType>& dst);
};

SOFA_DECL_CLASS(ConversionFunctor)

int ConversionFunctorClass =
		core::RegisterObject(
				"component apply alternative conversion algorithm when linked from PointVectorConverter")
				.add<ConversionFunctor>();

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_CONVERSIONFUNCTOR_H
