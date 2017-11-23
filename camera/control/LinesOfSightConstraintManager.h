#ifndef SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_H
#define SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaConstraint/SlidingConstraint.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofaor
{
namespace processor
{
template <class DataTypes>
class LOSConstraintManager : public sofaor::common::ImplicitDataEngine
{
  typedef typename sofa::component::constraintset::SlidingConstraint<
      DataTypes>::SPtr SlidingConstraint;
  typedef typename sofa::defaulttype::Vec2i Vec2i;
  typedef typename DataTypes::VecCoord VecCoord;
  typedef sofa::core::objectmodel::SingleLink<
      LOSConstraintManager, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef sofa::core::objectmodel::SingleLink<
      LOSConstraintManager,
      sofa::component::container::MechanicalObject<DataTypes>,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      MechanicalObject;

 public:
  SOFA_CLASS(SOFA_TEMPLATE(LOSConstraintManager, DataTypes),
             sofaor::common::ImplicitDataEngine);

 protected:
  CamSettings l_cam;
  MechanicalObject l_slavePoints;
  MechanicalObject l_masterPoints;
  sofa::Data<sofa::helper::vector<int> > d_indices;

  sofa::helper::vector<SlidingConstraint> m_components;

 public:
  LOSConstraintManager()
      : l_cam(initLink("cam", "camera to use for lines of sight projections")),
        l_slavePoints(initLink("slaveMO",
                               "MechanicalObject containing the position to "
                               "constrain on the lines of sight")),
        l_masterPoints(initLink(
            "masterMO",
            "MechanicalObject containing the position of the lines of sight")),
        d_indices(initData(
            &d_indices, "indices",
            "indices of the master points sorted by their matching slave"))
  {
  }

  virtual ~LOSConstraintManager() {}

  virtual void init();

  virtual void update();

  /// specific handleEvent behavior:
  /// Since input sources are links, update is not performed with standard
  /// handleEvent
  /// TODO: use positions vector only..?
  virtual void handleEvent(sofa::core::objectmodel::Event* /*e*/)
  {
    cleanInputs();
    update();
    setDirtyOutputs();
  }
};

}  // namespace processor
}  // namespace sofaor

#endif  // SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_H
