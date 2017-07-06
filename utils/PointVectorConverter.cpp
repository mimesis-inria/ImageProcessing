#include "PointVectorConverter.inl"

#include <SofaORCommon/cvKeypoint.h>

#include <sofa/core/ObjectFactory.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
SOFA_DECL_CLASS(PointVectorConverter)

int PointVectorConverterClass =
		sofa::core::RegisterObject(
				"Converts vector of cvKeyPoints to sofa vectors, and vice versa")
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, common::cvKeypoint> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec2d> >()
				.add<PointVectorConverter<common::cvKeypoint, sofa::defaulttype::Vec2i> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2d, common::cvKeypoint> >()
				.add<PointVectorConverter<common::cvKeypoint, sofa::defaulttype::Vec2d> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2d, sofa::defaulttype::Vec3d> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec3d> >();
}  // namespace utils
}  // namespace processor
}  // namespace sofaor
