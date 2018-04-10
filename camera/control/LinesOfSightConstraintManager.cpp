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
    #ifdef SOFA_FLOAT
        .add<LOSConstraintManager<sofa::defaulttype::Vec3fTypes> >()
    #endif // SOFA_FLOAT
    ;

template <class DataTypes>
LOSConstraintManager<DataTypes>::LOSConstraintManager()
    : l_cam(initLink("cam", "camera to use for lines of sight projections")),
      l_slavePoints(initLink("slaveMO",
                             "MechanicalObject containing the position to "
                             "constrain on the lines of sight")),
      l_masterPoints(initLink(
          "masterMO",
          "MechanicalObject containing the position of the lines of sight")),
      l_mapping(initLink("slaveBM", "BarycentricMapping used to map the slave MO")),
      d_indices(initData(
          &d_indices, "indices",
          "indices of the master points sorted by their matching slave"))
{
}

template <class DataTypes>
void LOSConstraintManager<DataTypes>::handleEvent(
    sofa::core::objectmodel::Event *e)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
  {
    update();
  }
}

}  // namespace processor
}  // namespace sofaor
