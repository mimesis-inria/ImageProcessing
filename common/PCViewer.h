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

#ifndef SOFACV_COMMON_PCVIEWER_H
#define SOFACV_COMMON_PCVIEWER_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace common
{
class SOFA_IMAGEPROCESSING_API PCViewer : public ImplicitDataEngine
{
	typedef sofa::defaulttype::Vector3 Vector3;
	typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;

 public:
    SOFA_CLASS(PCViewer, ImplicitDataEngine);

 public:
  PCViewer();
  virtual ~PCViewer();

  virtual void init() override;
  virtual void Update() override;
	void computeBBox(const sofa::core::ExecParams*, bool);
  virtual void draw(const sofa::core::visual::VisualParams*) override;
  virtual void reinit();

	sofa::Data<double> d_size;
	// INPUTS
	sofa::Data<sofa::helper::vector<Vector3> > d_positions;
	sofa::Data<sofa::helper::vector<Vec3b> > d_colors;
    sofa::Data<Vec3b> d_color;

 private:
	std::vector<sofa::defaulttype::Vec4f> m_colors;
	std::vector<Vector3> m_positions;
};

}  // namespace common
}  // namespace sofacv
#endif  // SOFACV_COMMON_PCVIEWER_H
