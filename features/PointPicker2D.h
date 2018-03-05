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

#ifndef SOFA_OR_PROCESSOR_POINTPICKER2D_H
#define SOFA_OR_PROCESSOR_POINTPICKER2D_H

#include "common/ImageFilter.h"

#include "camera/common/StereoSettings.h"

#include <sofa/helper/SVector.h>

namespace sofaor
{
namespace processor
{
namespace features
{
class PointPicker2D : public ImageFilter
{
  typedef sofa::core::objectmodel::SingleLink<
      PointPicker2D, cam::StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      Settings;

 public:
  SOFA_CLASS(PointPicker2D, ImageFilter);

  // INPUTS
  Settings l_cam;
  sofa::Data<int> d_whichImage;
  sofa::Data<std::string> d_getEpilinesFrom;
  // OUTPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_points;
  sofa::helper::vector<sofa::defaulttype::Vec3f> epilines;

  PointPicker2D();

  void init();

  virtual void Update() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

  void computeEpipolarLines();

 protected:
  PointPicker2D* m_picker;

  // Mouse controls
  typedef void (PointPicker2D::*StateFunction)(int, int, int, int);
  void freeMove(int event, int x, int y,
                int flags);  // mouse is moving, buttons are not pressed
  void capture(int event, int x, int y,
               int flags);  // left is down, capturing motion

  StateFunction m_activeState;
  void setMouseState(StateFunction f) { m_activeState = f; }
  void mouseCallback(int event, int x, int y, int flags);

 private:
  std::list<cv::Point2f> m_pointList;
};

SOFA_DECL_CLASS(PointPicker2D)

int PointPicker2DClass =
    sofa::core::RegisterObject("Manual 2D image point picker component")
        .add<PointPicker2D>();

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_POINTPICKER2D_H
