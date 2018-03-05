#ifndef SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_H
#define SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaBaseMechanics/BarycentricMapping.h>
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

    typedef sofa::core::objectmodel::SingleLink<
        LOSConstraintManager,
        sofa::core::objectmodel::BaseObject,
        sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
        BarycentricMapping;


 public:
  SOFA_CLASS(SOFA_TEMPLATE(LOSConstraintManager, DataTypes),
             sofaor::common::ImplicitDataEngine);

 protected:
  CamSettings l_cam;
  MechanicalObject l_slavePoints;
  MechanicalObject l_masterPoints;
  BarycentricMapping l_mapping;
  sofa::Data<sofa::helper::vector<int> > d_indices;

  sofa::helper::vector<SlidingConstraint> m_components;

 public:
  LOSConstraintManager();

  virtual ~LOSConstraintManager() {}

  virtual void init() override;

  virtual void Update() override;

  /// specific handleEvent behavior:
  /// Since input sources are links, update is not performed with standard
  /// handleEvent
  /// TODO: use positions vector only..?
  virtual void handleEvent(sofa::core::objectmodel::Event* /*e*/);
};

}  // namespace processor
}  // namespace sofaor

#endif  // SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_H
