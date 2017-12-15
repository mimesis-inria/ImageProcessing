/******************************************************************************
*       SOFAOR, SOFA plugin for the Operating Room, development version       *
*                        (c) 2017 INRIA, MIMESIS Team                         *
*                                                                             *
* This program is a free software; you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 1.0 of the License, or (at     *
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
* Authors: Bruno Marques and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact-mimesis@inria.fr                               *
******************************************************************************/

#ifndef SOFA_OR_ACQUISITOR_PCVIEWER_H
#define SOFA_OR_ACQUISITOR_PCVIEWER_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofaor
{
namespace processor
{
class PCViewer : public common::ImplicitDataEngine
{
	typedef sofa::defaulttype::Vector3 Vector3;
	typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;

 public:
	SOFA_CLASS(PCViewer, common::ImplicitDataEngine);

 public:
  PCViewer();
  virtual ~PCViewer();

  void init();
  void update();
	void computeBBox(const sofa::core::ExecParams*, bool);
	void draw(const sofa::core::visual::VisualParams*);
  void reinit();

	sofa::Data<double> d_size;
	// INPUTS
	sofa::Data<sofa::helper::vector<Vector3> > d_positions;
	sofa::Data<sofa::helper::vector<Vec3b> > d_colors;
    sofa::Data<Vec3b> d_color;

 private:
	std::vector<sofa::defaulttype::Vec4f> m_colors;
	std::vector<Vector3> m_positions;
};

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_ACQUISITOR_PCVIEWER_H
