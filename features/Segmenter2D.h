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

#ifndef SOFA_OR_PROCESSOR_SEGMENTER2D_H
#define SOFA_OR_PROCESSOR_SEGMENTER2D_H

#include "common/ImageFilter.h"
#include <sofa/helper/SVector.h>

namespace sofaor
{
namespace processor
{
namespace features
{
class Segmenter2D : public ImageFilter
{
 public:
  SOFA_CLASS(Segmenter2D, ImageFilter);

  sofa::Data<std::string> d_regionLabel;

  // INPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_points;

  // OUTPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_regionPoly;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_regionPoints;

  Segmenter2D();

  virtual void init() override;

  virtual void Update() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

  // Mouse controls
  typedef void (Segmenter2D::*StateFunction)(int, int, int, int);
  void freeMove(int event, int x, int y,
                int flags);  // mouse is moving, buttons are not pressed
  void capture(int event, int x, int y,
               int flags);  // left is down, capturing motion
  void capturePaused(int event, int x, int y,
                     int flags);  // left is down, capturing motion
  void stopping(int event, int x, int y, int flags);  // right is down, stopping

  StateFunction m_activeState;
  void setMouseState(StateFunction f) { m_activeState = f; }
  void mouseCallback(int event, int x, int y, int flags);

  std::list<cv::Point2i> m_poly;
};

SOFA_DECL_CLASS(Segmenter2D)

int Segmenter2DClass =
    sofa::core::RegisterObject("Manual segmentation component")
        .add<Segmenter2D>();

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_SEGMENTER2D_H
