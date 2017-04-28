#include "Vector2SSVector.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(Vector2SSVector)

int Vector2SSVectorClass =
		core::RegisterObject("Converts vector to vectors of vectors")
				.add<Vector2SSVector<defaulttype::Vec2i> >()
				.add<Vector2SSVector<common::cvKeypoint> >()
				.add<Vector2SSVector<defaulttype::Vec2f> >()
				.add<Vector2SSVector<defaulttype::Vec3f> >()
				.add<Vector2SSVector<defaulttype::Vec2d> >()
				.add<Vector2SSVector<defaulttype::Vec3d> >();
}  // namespace processor
}  // namespace OR
}  // namespace sofa
