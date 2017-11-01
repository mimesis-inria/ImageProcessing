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

#include "Segmenter2D.h"
#include <opencv2/highgui.hpp>
namespace sofaor
{
namespace processor
{
namespace features
{
/// Simple Mouse FSM:
///        LDown     +--------+     RUp
///          +-------|freeMove|<-------+
///          |       +--------+        |
///          |                         |
///          v                         |
///      +-------+                 +---+---+
///      |capture|                 |stopped|
///      +-+---+-+                 +-+---+-+
///        |   ^ LDown         RDown |   ^
///        |   +------+------+<------+   |
///        |   LUp    |paused|  !RUp     |
///        +--------->+------+-----------+
///

void Segmenter2D::freeMove(int event, int /*x*/, int /*y*/, int /*flags*/)
{
	if (event == cv::EVENT_LBUTTONDOWN)
    setMouseState(&Segmenter2D::capture);
  else if (event == cv::EVENT_MBUTTONDOWN)
  {
    d_regionPoly.beginWriteOnly()->clear();
    d_regionLabel.beginWriteOnly()->clear();
    d_regionPoly.endEdit();
    d_regionLabel.endEdit();
    ImageFilter::update();
  }
}
void Segmenter2D::capture(int event, int x, int y, int flags)
{
	switch (event)
  {
    case cv::EVENT_MOUSEMOVE:
    {
      cv::Point2i pos(x, y);
      if (flags & cv::EVENT_FLAG_CTRLKEY)
      {
        for (int _x = 0; _x < 6; ++_x)
          for (int _y = 0; _y < 6; ++_y)
          {
            pos = cv::Point2i(x - 2 + _x, y - 2 + _y);
            m_poly.remove(pos);
          }
      }
      else
        m_poly.push_back(pos);
    }
    break;
    case cv::EVENT_LBUTTONUP:
      cv::Point2i pos(x, y);
      if (flags & cv::EVENT_FLAG_CTRLKEY)
      {
        for (int _x = 0; _x < 6; ++_x)
          for (int _y = 0; _y < 6; ++_y)
          {
            pos = cv::Point2i(x - 2 + _x, y - 2 + _y);
            m_poly.remove(pos);
          }
      }
      else
        m_poly.push_back(pos);
      setMouseState(&Segmenter2D::capturePaused);
      break;
  }
  update();
}

void Segmenter2D::capturePaused(int event, int /*x*/, int /*y*/, int flags)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		if (flags & cv::EVENT_FLAG_SHIFTKEY)
			setMouseState(&Segmenter2D::stopping);
		else
			setMouseState(&Segmenter2D::capture);
  }
}

void Segmenter2D::stopping(int event, int /*x*/, int /*y*/, int /*flags*/)
{
	switch (event)
  {
		case cv::EVENT_LBUTTONUP:
    {
			std::cout << "STOPPED" << std::endl;
			sofa::helper::vector<sofa::defaulttype::Vec2i>* regionsPoly =
          d_regionPoly.beginEdit();
			regionsPoly->clear();
			regionsPoly->reserve(m_poly.size());
      for (const cv::Point2i& pt : m_poly)
			{
				regionsPoly->push_back(sofa::defaulttype::Vec2i(pt.x, pt.y));
				std::cout << pt.x << " " << pt.y << " ";
			}
			std::cout << std::endl << std::endl;
			d_regionPoly.endEdit();
      m_poly.clear();
      setMouseState(&Segmenter2D::freeMove);
      ImageFilter::update();
    }
    break;
    default:
      setMouseState(&Segmenter2D::capturePaused);
      break;
  }
}

void Segmenter2D::mouseCallback(int event, int x, int y, int flags)
{
    if (d_outputImage.getValue())
        (this->*m_activeState)(event, x, y, flags);
}

}  // namespace features
}  // namespace processor
}  // namespace sofaor
