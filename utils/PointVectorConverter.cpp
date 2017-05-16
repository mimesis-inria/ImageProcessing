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
				.add<PointVectorConverter<defaulttype::Vec2i, defaulttype::Vec2d> >()
				.add<PointVectorConverter<common::cvKeypoint, defaulttype::Vec2i> >()
				.add<PointVectorConverter<defaulttype::Vec2d, common::cvKeypoint> >()
				.add<PointVectorConverter<common::cvKeypoint, defaulttype::Vec2d> >()
				.add<PointVectorConverter<defaulttype::Vec2d, defaulttype::Vec3d> >()
				.add<PointVectorConverter<defaulttype::Vec2i, defaulttype::Vec3d> >();
}  // namespace processor
}  // namespace OR
}  // namespace sofa
