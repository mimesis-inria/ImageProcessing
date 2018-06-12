#include "PointVectorConverter.inl"

#include <SofaCV/SofaCV.h>

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace utils
{
SOFA_DECL_CLASS(PointVectorConverter)

int PointVectorConverterClass =
		sofa::core::RegisterObject(
				"Converts vector of cvKeyPoints to sofa vectors, and vice versa")
                .add<PointVectorConverter<sofa::defaulttype::Vec2i, cvKeypoint> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec2d> >()
                .add<PointVectorConverter<cvKeypoint, sofa::defaulttype::Vec2i> >()
                .add<PointVectorConverter<sofa::defaulttype::Vec2d, cvKeypoint> >()
                .add<PointVectorConverter<cvKeypoint, sofa::defaulttype::Vec2d> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2d, sofa::defaulttype::Vec3d> >()
				.add<PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec3d> >();
}  // namespace utils
}  // namespace sofacv
