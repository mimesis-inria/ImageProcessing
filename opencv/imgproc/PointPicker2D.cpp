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
  std::cout << "freemove" << std::endl;
  if (event == cv::EVENT_LBUTTONDOWN)
    setMouseState(&PointPicker2D::capture);
  else if (event == cv::EVENT_RBUTTONDOWN)
  {
    d_points.beginWriteOnly()->clear();
    d_points.endEdit();
    ImageFilter::update();
  }
}
void PointPicker2D::capture(int event, int x, int y, int flags)
{
  std::cout << "capture" << std::endl;
  switch (event)
  {
    case cv::EVENT_MOUSEMOVE:
    {
      setMouseState(&PointPicker2D::freeMove);
    }
    break;
    case cv::EVENT_LBUTTONUP:
    {
      defaulttype::Vec2i pos(x, y);
      if (flags & cv::EVENT_FLAG_CTRLKEY)
      {
        for (int _x = 0; _x < 6; ++_x)
          for (int _y = 0; _y < 6; ++_y)
          {
            pos = defaulttype::Vec2i(x - 2 + _x, y - 2 + _y);
//            d_points.beginWriteOnly()->erase(pos);
          }
        std::cout << "removing" << std::endl;
      }
      else
        d_points.beginWriteOnly()->push_back(pos);
      d_points.endEdit();
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
