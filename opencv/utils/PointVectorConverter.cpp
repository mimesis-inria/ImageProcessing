#include "PointVectorConverter.inl"

#include <SofaORCommon/cvKeypoint.h>

#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(PointVectorConverter)

int PointVectorConverterClass =
		core::RegisterObject(
				"Converts vector of cvKeyPoints to sofa vectors, and vice versa")
				.add<PointVectorConverter<defaulttype::Vec2i, common::cvKeypoint> >()
				.add<PointVectorConverter<common::cvKeypoint, defaulttype::Vec2i> >()
				.add<PointVectorConverter<defaulttype::Vec2f, common::cvKeypoint> >()
				.add<PointVectorConverter<common::cvKeypoint, defaulttype::Vec2f> >();
}  // namespace processor
}  // namespace OR
}  // namespace sofa
