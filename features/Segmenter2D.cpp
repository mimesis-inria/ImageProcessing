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
#include <opencv2/imgproc.hpp>

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

Segmenter2D::Segmenter2D()
    : d_regionLabel(
          initData(&d_regionLabel, "label", "label for the segmented region")),
      d_points(initData(&d_points, "points", "input vector keypoints")),
      d_regionPoly(initData(&d_regionPoly, "poly", "optional input polygon")),
      d_regionPoints(initData(&d_regionPoints, "points_out",
                              "output vector of points fitting in the poly"))
{
  addAlias(&d_regionPoly, "poly_out");
}

void Segmenter2D::init()
{
  addInput(&d_points);
  addInput(&d_regionPoly);
  addOutput(&d_regionPoints);
  ImageFilter::activateMouseCallback();
  setMouseState(&Segmenter2D::freeMove);
  ImageFilter::init();
  update();
}

void Segmenter2D::update()
{
  ImageFilter::update();

  if (d_regionPoly.getValue().size() <= 2)
  {
    d_regionPoints.setValue(d_points.getValue());
    return;
  }

  std::vector<cv::Point2i> polygon;
  for (const sofa::defaulttype::Vec2i& pt : d_regionPoly.getValue())
  {
    polygon.push_back(cv::Point2i(pt.x(), pt.y()));
  }
  sofa::helper::vector<sofa::defaulttype::Vec2i>& points =
      *d_regionPoints.beginWriteOnly();
  points.clear();

  const sofa::helper::vector<sofa::defaulttype::Vec2i>& pts =
      d_points.getValue();

  for (auto point : pts)
  {
    cv::Point2f pt(point.x(), point.y());
    if (cv::pointPolygonTest(polygon, pt, false) > 0) points.push_back(point);
  }
  d_regionPoints.endEdit();
}

void Segmenter2D::applyFilter(const cv::Mat& in, cv::Mat& out, bool debug)
{
  if (in.empty()) return;
  in.copyTo(out);

  std::vector<std::vector<cv::Point2i> > polygon(1);
  if (m_activeState == &Segmenter2D::freeMove)
  {
    if (d_regionPoly.getValue().empty())
    {
      cv::putText(out, "- LClick: add point", cv::Point(15, out.rows - 95),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0));
      cv::putText(out, "- Left + move: draw", cv::Point(15, out.rows - 75),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0));
      cv::putText(out, "- Ctrl + LClick: del point",
                  cv::Point(15, out.rows - 55), cv::FONT_HERSHEY_COMPLEX_SMALL,
                  1.0, CV_RGB(0, 255, 0));
      cv::putText(out, "- Ctrl + move: undraw", cv::Point(15, out.rows - 35),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0));
      cv::putText(out, "- MClick: delete all", cv::Point(15, out.rows - 15),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, CV_RGB(0, 255, 0));
      return;
    }

    for (const sofa::defaulttype::Vec2i& pt : d_regionPoly.getValue())
    {
      polygon[0].push_back(cv::Point2i(pt.x(), pt.y()));
    }
  }
  else
  {
    polygon[0] = std::vector<cv::Point2i>(m_poly.begin(), m_poly.end());
  }
  cv::Scalar color(255, 255, 255, 50);
  cv::fillPoly(out, polygon, color);
  cv::addWeighted(in, 0.8, out, 0.2, 0.0, out);
}

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
  if (d_outputImage.getValue()) (this->*m_activeState)(event, x, y, flags);
}

}  // namespace features
}  // namespace processor
}  // namespace sofaor
