#ifndef SOFA_OR_PROCESSOR_NEGATEVECTOR_H
#define SOFA_OR_PROCESSOR_NEGATEVECTOR_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
class NegateVector : public common::ImplicitDataEngine
{
 public:
	SOFA_CLASS(NegateVector, common::ImplicitDataEngine);

	NegateVector()
			: d_src(initData(&d_src, "input_position", "input vector to negate")),
				d_srcRot(
						initData(&d_srcRot, "input_rotation", "input rotation to negate")),
				d_dst(initData(&d_dst, "output_position", "negated output vector")),
				d_dstRot(
						initData(&d_dstRot, "output_rotation", "negated output vector"))
	{
	}

	~NegateVector() {}
	void init()
	{
		addInput(&d_src);
		addInput(&d_srcRot);
		addOutput(&d_dst);
	}

	void update()
	{
		d_dst.setValue(-d_src.getValue());
		sofa::defaulttype::Matrix3 rev;
		d_dstRot.setValue(-d_srcRot.getValue().transposed());
		d_dst.setValue(d_dstRot.getValue() * d_dst.getValue());
	}

	// INPUTS
	sofa::Data<sofa::defaulttype::Vector3> d_src;
	sofa::Data<sofa::defaulttype::Matrix3> d_srcRot;
	// OUTPUTS
	sofa::Data<sofa::defaulttype::Vector3> d_dst;
	sofa::Data<sofa::defaulttype::Matrix3> d_dstRot;
};

SOFA_DECL_CLASS(NegateVector)

int NegateVectorClass =
		sofa::core::RegisterObject("component to negate a defaulttype::Vector3")
				.add<NegateVector>();

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_NEGATEVECTOR_H
