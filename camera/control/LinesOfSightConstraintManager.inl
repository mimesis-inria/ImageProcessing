#ifndef SOFACV_CAM_CONTROL_LINESOFSIGHTCONSTRAINT_INL
#define SOFACV_CAM_CONTROL_LINESOFSIGHTCONSTRAINT_INL

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "LinesOfSightConstraintManager.h"

namespace sofacv
{
namespace cam
{
namespace control
{
template <class DataTypes>
void LOSConstraintManager<DataTypes>::init()
{
  addInput(&d_indices);
  if (!l_slavePoints.get())
    msg_error(getName() + "::init()") << "No MState provided for slave points!";
  if (!l_masterPoints.get())
    msg_error(getName() + "::init()")
        << "No MState provided for master points!";
  m_components = sofa::helper::vector<SlidingConstraint>(
      d_indices.getValue().size() + 1, nullptr);
  setDirtyValue();
  update();
}

template <class DataTypes>
void LOSConstraintManager<DataTypes>::createConstraints()
{
  // Retrieve all constraints in current node and remove them
  std::vector<sofa::component::constraintset::SlidingConstraint<DataTypes>*>
      engines;
  sofa::core::objectmodel::BaseContext* ctx = this->getContext();
  ctx->get<sofa::component::constraintset::SlidingConstraint<DataTypes> >(
      &engines);
  for (sofa::component::constraintset::SlidingConstraint<DataTypes>*& c :
       engines)
  {
    c->cleanup();
    ctx->removeObject(c);
  }

  m_components.clear();

  // Recreate Constraints
  if (l_masterPoints.get() == nullptr || l_slavePoints.get() == nullptr)
  {
    msg_error(getName())
        << "Mechanical states not linked properly or deleted during simulation";
    return;
  }

  if (d_indices.isSet())
  {
    for (size_t i = 0; i < d_indices.getValue().size(); ++i)
    {
      m_components.push_back(nullptr);
      sofa::core::objectmodel::BaseObjectDescription desc(
          (std::string("SlidingConstraint_") +
           std::to_string(d_indices.getValue()[i]))
              .c_str(),
          "SlidingConstraint");
      desc.setAttribute("name", (std::string("SlidingConstraint_") +
                                 std::to_string(d_indices.getValue()[i]))
                                    .c_str());
      desc.setAttribute("object1", l_slavePoints.getPath().c_str());
      desc.setAttribute("object2", l_masterPoints.getPath().c_str());
      desc.setAttribute("sliding_point",
                        std::to_string(d_indices.getValue()[i]).c_str());
      desc.setAttribute("axis_1", std::to_string(i).c_str());
      desc.setAttribute("axis_2",
                        std::to_string(l_masterPoints->x.getValue().size() - 1)
                            .c_str());  // cam position index in masterPoints

      //      std::cout << "LOSConstraint: "
      //                << l_slavePoints->x.getValue()[d_indices.getValue()[i]]
      //                << "\t"
      //                << l_masterPoints->x.getValue()[i] << std::endl;
      m_components[i] = sofa::core::objectmodel::New<
          sofa::component::constraintset::SlidingConstraint<DataTypes> >();
      this->getContext()->addObject(m_components[i]);
      m_components[i]->parse(&desc);
      m_components[i]->init();
      m_components[i]->reinit();
      m_components[i]->bwdInit();
    }
    //    std::cout << std::endl;
  }
  else
  {
    if (l_slavePoints->x.getValue().size() !=
        l_masterPoints->x.getValue().size() - 1)
    {
      msg_error(this) << "slavePoints and masterPoints must have the same size";
      return;
    }
    for (size_t i = 0; i < l_slavePoints->x.getValue().size(); ++i)
    {
      m_components.push_back(nullptr);
      sofa::core::objectmodel::BaseObjectDescription desc(
          (std::string("SlidingConstraint_") + std::to_string(i)).c_str(),
          "SlidingConstraint");
      desc.setAttribute(
          "name",
          (std::string("SlidingConstraint_") + std::to_string(i)).c_str());
      desc.setAttribute("object1", l_slavePoints.getPath().c_str());
      desc.setAttribute("object2", l_masterPoints.getPath().c_str());
      desc.setAttribute("sliding_point", std::to_string(i).c_str());
      desc.setAttribute("axis_1", std::to_string(i).c_str());
      desc.setAttribute("axis_2",
                        std::to_string(l_masterPoints->x.getValue().size() - 1)
                            .c_str());  // cam position index in masterPoints

      m_components[i] = sofa::core::objectmodel::New<
          sofa::component::constraintset::SlidingConstraint<DataTypes> >();
      this->getContext()->addObject(m_components[i]);
      m_components[i]->parse(&desc);
      m_components[i]->init();
      m_components[i]->reinit();
      m_components[i]->bwdInit();
    }
  }
}

template <class DataTypes>
void LOSConstraintManager<DataTypes>::Update()
{
  if (d_rebuildConstraints.getValue() /* || firstPass*/)
  {
    l_slavePoints->reinit();
    l_masterPoints->reinit();
    createConstraints();
  }
}

template <class DataTypes>
void LOSConstraintManager<DataTypes>::handleEvent(
    sofa::core::objectmodel::Event* event)
{
  if (sofa::simulation::AnimateBeginEvent::checkEventType(event))
  {
    this->update();
  }
}
}  // namespace control
}  // namespace cam
}  // namespace sofacv

#endif  // SOFACV_CAM_CONTROL_LINESOFSIGHTCONSTRAINT_INL
