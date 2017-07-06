#ifndef SOFA_OR_PROCESSOR_VECTOR2SSVECTOR_H
#define SOFA_OR_PROCESSOR_VECTOR2SSVECTOR_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

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

	void update();

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
