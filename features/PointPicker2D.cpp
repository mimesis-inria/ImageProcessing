#include "PointPicker2D.h"
#include <SofaORCommon/cvMatUtils.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

namespace sofa
{
namespace OR
{
namespace processor
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
	if (!m_picker && !l_cam.get()) return;
	std::vector<cv::Vec3f> lines;
	epilines.clear();
	if (!m_pointList.empty())
	{
		cv::Mat_<double> F;
		common::matrix::sofaMat2cvMat(l_cam->getFundamentalMatrix(), F);
		std::vector<cv::Point2f> points(m_pointList.begin(), m_pointList.end());
		cv::computeCorrespondEpilines(points, d_whichImage.getValue(), F, lines);
		for (const cv::Vec3f& pt : lines)
			epilines.push_back(defaulttype::Vec3f(pt.val));
	}
	m_picker->reinitDebugWindow();
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

}  // namespace processor
}  // namespace OR
}  // namespace sofa
