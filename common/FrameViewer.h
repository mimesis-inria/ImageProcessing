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

#ifndef SOFACV_COMMON_FRAMEVIEWER_H
#define SOFACV_COMMON_FRAMEVIEWER_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>

#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/Quater.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace common
{
class SOFA_IMAGEPROCESSING_API FrameViewer : virtual public ImplicitDataEngine
{
 public:
    SOFA_CLASS(FrameViewer, ImplicitDataEngine);

 public:
  FrameViewer();
  virtual ~FrameViewer();

  void init() override;
  void reinit() override;
  void Update() override;
  void draw(const sofa::core::visual::VisualParams*) override;
  void computeBBox(const sofa::core::ExecParams* params, bool) override;

    sofa::Data<cvMat> d_frame;
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector3> > d_corners;
	sofa::Data<sofa::helper::OptionsGroup> d_mode;
	sofa::Data<float> d_alpha;

 private:
	void perspectiveDraw();
	void orthoDraw();
	void bindGlTexture(const std::string& imageString);
    void drawImpl();
};

}  // namespace processor
}  // namespace sofaor
#endif  // SOFACV_COMMON_FRAMEVIEWER_H
