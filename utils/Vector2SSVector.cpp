#include "Vector2SSVector.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
SOFA_DECL_CLASS(Vector2SSVector)

int Vector2SSVectorClass =
		sofa::core::RegisterObject("Converts vector to vectors of vectors")
				.add<Vector2SSVector<sofa::defaulttype::Vec2i> >()
				.add<Vector2SSVector<common::cvKeypoint> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec2f> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec3f> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec2d> >()
				.add<Vector2SSVector<sofa::defaulttype::Vec3d> >();
}  // namespace utils
}  // namespace processor
}  // namespace sofaor
