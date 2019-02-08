/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_FUSION_AddCam_H
#define SOFA_FUSION_AddCam_H

#include <SofaBaseMechanics/BarycentricMapping.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaCV/SofaCV.h>
#include <sofa/defaulttype/Vec3Types.h>

namespace sofa
{
namespace component
{
namespace engine
{
template <class DataTypes>
class AddCam : public sofacv::ImplicitDataEngine
{
  typedef typename DataTypes::Coord Coord;
  typedef typename DataTypes::VecCoord VecCoord;

 public:
  SOFA_CLASS(AddCam, sofacv::ImplicitDataEngine);

  AddCam()
      : d_input_position(initData(&d_input_position, "input_position",
                                  "input position")),
        d_output_position(initData(&d_output_position, "output_position",
                                   "output position")),
        d_camPos(
            initData(&d_camPos, "camPos", "camera position to add to vector"))
  {
  }

  virtual ~AddCam() {}

 public:
  void init() override
  {
    addInput(&d_input_position);
    addInput(&d_camPos);
    addOutput(&d_output_position);
    setDirtyValue();
    update();
  }

  void Update() override
  {
    VecCoord pos = d_input_position.getValue();

    if (!d_camPos.isSet())
    {
        msg_error(this) << "You forgot to provide camera to link!";
        return;
    }
    pos.push_back(d_camPos.getValue());

    cleanDirty();
    d_output_position.setValue(pos);
  }

  Data<VecCoord> d_input_position;
  Data<VecCoord> d_output_position;
  Data<Coord> d_camPos;
};

SOFA_DECL_CLASS(AddCam)

int AddCamClass =
    sofa::core::RegisterObject("shuffles a vector of points")
        .add<AddCam<defaulttype::Vec3dTypes> >();

}  // namespace engine

}  // namespace component

}  // namespace sofa

#endif  // AddCam
