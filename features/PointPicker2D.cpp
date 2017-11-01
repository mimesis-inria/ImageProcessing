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

#include "PointPicker2D.h"
#include <SofaORCommon/cvMatUtils.h>
#include <opencv2/calib3d.hpp>
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

void PointPicker2D::computeEpipolarLines()
{
	if (!m_picker || !l_cam.get() || l_cam->getFundamentalMatrix().empty())
		return;
	std::vector<cv::Vec3f> lines;
	epilines.clear();
	if (!m_pointList.empty())
	{
		cv::Mat_<double> F;
		common::matrix::sofaMat2cvMat(l_cam->getFundamentalMatrix(), F);
		std::vector<cv::Point2f> points(m_pointList.begin(), m_pointList.end());
		cv::computeCorrespondEpilines(points, d_whichImage.getValue(), F, lines);
		for (const cv::Vec3f& pt : lines)
			epilines.push_back(sofa::defaulttype::Vec3f(pt.val));
	}
	m_picker->refreshDebugWindow();
}

void PointPicker2D::freeMove(int event, int /*x*/, int /*y*/, int /*flags*/)
{
	if (event == cv::EVENT_LBUTTONDOWN)
    setMouseState(&PointPicker2D::capture);
	else if (event == cv::EVENT_MBUTTONDOWN)
  {
    m_pointList.clear();
		computeEpipolarLines();
		update();
  }
}

void PointPicker2D::capture(int event, int x, int y, int flags)
{
  switch (event)
  {
    case cv::EVENT_MOUSEMOVE:
    {
      setMouseState(&PointPicker2D::freeMove);
    }
    break;
    case cv::EVENT_LBUTTONUP:
    {
			cv::Point2f pos(x, y);
      if (flags & cv::EVENT_FLAG_CTRLKEY)
      {
        for (int _x = 0; _x < 6; ++_x)
          for (int _y = 0; _y < 6; ++_y)
          {
						pos = cv::Point2f(x - 2 + _x, y - 2 + _y);
            m_pointList.remove(pos);
          }
      }
      else
        m_pointList.push_back(pos);

			computeEpipolarLines();

			setMouseState(&PointPicker2D::freeMove);
      break;
    }
  }
  update();
}

void PointPicker2D::mouseCallback(int event, int x, int y, int flags)
{
  (this->*m_activeState)(event, x, y, flags);
}

}  // namespace features
}  // namespace processor
}  // namespace sofaor
