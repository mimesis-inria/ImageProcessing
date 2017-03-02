#include "PointPicker2D.h"
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

void PointPicker2D::freeMove(int event, int x, int y, int flags)
{
  if (event == cv::EVENT_LBUTTONDOWN)
    setMouseState(&PointPicker2D::capture);
  else if (event == cv::EVENT_RBUTTONDOWN)
  {
    m_pointList.clear();
    ImageFilter::update();
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
      cv::Point2i pos(x, y);
      if (flags & cv::EVENT_FLAG_CTRLKEY)
      {
        for (int _x = 0; _x < 6; ++_x)
          for (int _y = 0; _y < 6; ++_y)
          {
            pos = cv::Point2i(x - 2 + _x, y - 2 + _y);
            m_pointList.remove(pos);
          }
        std::cout << "removing" << std::endl;
      }
      else
        m_pointList.push_back(pos);
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
