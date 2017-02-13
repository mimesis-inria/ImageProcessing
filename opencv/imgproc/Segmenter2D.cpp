#include "Segmenter2D.h"
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

void Segmenter2D::freeMove(int event, int x, int y, int flags)
{
    std::cout << "freemove" << std::endl;
  if (event == cv::EVENT_LBUTTONDOWN)
    setMouseState(&Segmenter2D::capture);
  else if (event == cv::EVENT_MBUTTONDOWN)
  {
    d_regionPoly.beginWriteOnly()->clear();
    d_regionLabel.beginWriteOnly()->clear();
    d_regionPoly.endEdit();
    d_regionLabel.endEdit();
    update();
  }
}
void Segmenter2D::capture(int event, int x, int y, int flags)
{
    std::cout << "capture" << std::endl;
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
        std::cout << "removing" << std::endl;
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
        std::cout << "removing" << std::endl;
      }
      else
        m_poly.push_back(pos);
      setMouseState(&Segmenter2D::capturePaused);
      break;
  }
  update();
}

void Segmenter2D::capturePaused(int event, int x, int y, int flags)
{
    std::cout << "paused" << std::endl;
  switch (event)
  {
    case cv::EVENT_LBUTTONDOWN:
      setMouseState(&Segmenter2D::capture);
      break;
    case cv::EVENT_RBUTTONDOWN:
      setMouseState(&Segmenter2D::stopping);
      break;
  }
}

void Segmenter2D::stopping(int event, int x, int y, int flags)
{
    std::cout << "stopping" << std::endl;
  switch (event)
  {
    case cv::EVENT_RBUTTONUP:
    {
      helper::SVector<helper::SVector<defaulttype::Vec2i> >* regionsPoly =
          d_regionPoly.beginEdit();
      helper::vector<defaulttype::Vec2i> tmp;
      tmp.reserve(m_poly.size());
      for (const cv::Point2i& pt : m_poly)
        tmp.push_back(defaulttype::Vec2i(pt.x, pt.y));
      regionsPoly->push_back(tmp);
      d_regionPoly.endEdit();
      d_regionLabel.beginWriteOnly()->push_back("regionName");
      m_poly.clear();
      setMouseState(&Segmenter2D::freeMove);
      update();
    }
    break;
    default:
      setMouseState(&Segmenter2D::capturePaused);
      break;
  }
}

void Segmenter2D::mouseCallback(int event, int x, int y, int flags)
{
  (this->*m_activeState)(event, x, y, flags);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
