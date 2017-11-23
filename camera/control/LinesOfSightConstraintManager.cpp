#include "LinesOfSightConstraintManager.inl"

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/Vec3Types.h>

namespace sofaor
{
namespace processor
{

SOFA_DECL_CLASS(LOSConstraintManager)

int LOSConstraintManagerClass =
    sofa::core::RegisterObject("Lines of sight Constraint Manager")
        .add<LOSConstraintManager<sofa::defaulttype::Vec3dTypes> >(true)
        .add<LOSConstraintManager<sofa::defaulttype::Vec3fTypes> >()
    ;

}  // namespace processor
}  // namespace sofaor
