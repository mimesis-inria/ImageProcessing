#ifndef SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_INL
#define SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_INL

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "LinesOfSightConstraintManager.h"

namespace sofaor
{
namespace processor
{
template <class DataTypes>
void LOSConstraintManager<DataTypes>::init()
{
  srand(time(NULL));
  if (!l_cam) msg_error(getName() + "::init()") << "No Camera link set!";
  if (!l_slavePoints.get())
    msg_error(getName() + "::init()") << "No MState provided for slave points!";
  if (!l_masterPoints.get())
    msg_error(getName() + "::init()")
        << "No MState provided for master points!";
  addInput(&d_indices);
  m_components = sofa::helper::vector<SlidingConstraint>(
      d_indices.getValue().size() + 1, nullptr);
  update();
}

template <class DataTypes>
void LOSConstraintManager<DataTypes>::update()
{
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

  std::vector<int> KeptIndices;
  m_components.clear();

  // downsampling according to d_maxConstraints

  for (int i = 0; i < d_indices.getValue().size(); ++i)
    if (!d_maxConstraints.getValue() || i % d_maxConstraints.getValue() == 0)
    {
      KeptIndices.push_back(i);
      m_components.push_back(nullptr);
    }

  if (l_masterPoints.get() == nullptr) return;

  // Before creating the constraints, reinit the Barycentric mapper:
  l_mapping->reinit();
  l_mapping->bwdInit();

  for (const auto& i : KeptIndices)
  {
    sofa::core::objectmodel::BaseObjectDescription desc(
        (std::string("SlidingConstraint_") + std::to_string(i)).c_str(),
        "SlidingConstraint");
    desc.setAttribute(
        "name",
        (std::string("SlidingConstraint_") + std::to_string(i)).c_str());
    desc.setAttribute("object1", l_slavePoints.getPath().c_str());
    desc.setAttribute("object2", l_masterPoints.getPath().c_str());
    desc.setAttribute("sliding_point", std::to_string(i).c_str());
    desc.setAttribute("axis_1",
                      std::to_string(d_indices.getValue()[i]).c_str());

    desc.setAttribute(
        "axis_2",
        std::to_string(l_masterPoints->xfree.getValue().size() - 1)
            .c_str());  // l_cam->position's index in masterPoints

    if (m_components[i].get() == nullptr)
    {
      m_components[i] = sofa::core::objectmodel::New<
          sofa::component::constraintset::SlidingConstraint<DataTypes> >();
      this->getContext()->addObject(m_components[i]);
    }
    m_components[i]->parse(&desc);
    m_components[i]->init();
  }
}

}  // namespace processor
}  // namespace sofaor

#endif  // SOFAOR_PROCESSOR_CAMERA_CONTROL_LINESOFSIGHTCONSTRAINT_INL
