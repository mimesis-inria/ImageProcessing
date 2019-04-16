#ifndef SOFACV_CAM_CONTROL_LINESOFSIGHTCONSTRAINT_H
#define SOFACV_CAM_CONTROL_LINESOFSIGHTCONSTRAINT_H

#include "ImageProcessing/ImageProcessingPlugin.h"
#include "ImageProcessing/camera/common/CameraSettings.h"

#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaConstraint/SlidingConstraint.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/objectmodel/BaseObjectDescription.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofacv
{
namespace cam
{
namespace control
{
template <class DataTypes>
class SOFA_IMAGEPROCESSING_API LOSConstraintManager : public ImplicitDataEngine
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
      LOSConstraintManager, sofa::core::objectmodel::BaseObject,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      BarycentricMapping;

 public:
  SOFA_CLASS(SOFA_TEMPLATE(LOSConstraintManager, DataTypes),
             ImplicitDataEngine);

 protected:
  CamSettings l_cam;
  MechanicalObject l_slavePoints;
  MechanicalObject l_masterPoints;
  BarycentricMapping l_mapping;
  sofa::Data<sofa::helper::vector<int> > d_indices;

  sofa::helper::vector<SlidingConstraint> m_components;
  sofa::Data<bool> d_rebuildConstraints;

 public:
  LOSConstraintManager();

  virtual ~LOSConstraintManager() override {}

  virtual void init() override;

  virtual void doUpdate() override;

  void createConstraints();

  virtual void handleEvent(sofa::core::objectmodel::Event* /*e*/) override;
};

}  // namespace control
}  // namespace cam
}  // namespace sofacv

#endif  // SOFACV_CAM_CONTROL_LINESOFSIGHTCONSTRAINT_H
