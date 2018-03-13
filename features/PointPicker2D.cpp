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

PointPicker2D::PointPicker2D()
    : l_cam(initLink("cam",
                     "optional input StereoSettings component to get the "
                     "fundamental matrix from (used to compute epipolar "
                     "lines")),
      d_whichImage(initData(&d_whichImage, "whichImage",
                            "optional input integer to define if it's Left "
                            "(1) or Right(2) image, to compute epipolar "
                            "lines")),
      d_getEpilinesFrom(initData(
          &d_getEpilinesFrom, "getEpilinesFrom",
          "optional input component from which to look for epipolar lines")),
      d_points(initData(&d_points, "points",
                        "output vector of 2D points picked in the image", true,
                        true))
{
  addAlias(&d_points, "points_out");
}

void PointPicker2D::init()
{
  addInput(&d_whichImage);
  addOutput(&d_points);
  ImageFilter::activateMouseCallback();
  setMouseState(&PointPicker2D::freeMove);
  ImageFilter::init();
  m_picker =
      this->getContext()->get<PointPicker2D>(d_getEpilinesFrom.getValue());
  if (!d_points.getValue().empty())
    for (auto pt : d_points.getValue())
      m_pointList.push_back(cv::Point2f(pt.x(), pt.y()));

  if (m_picker && !l_cam.get())
    msg_advice(getName() + "::init()")
        << "No Stereo camera settings link set. "
           "If you want to visualize the epipolar lines, this is necessary";
}

void PointPicker2D::update()
{
  ImageFilter::update();
  sofa::helper::vector<sofa::defaulttype::Vec2i>* points =
      d_points.beginWriteOnly();
  points->clear();
  if (!m_pointList.empty())
  {
    std::cout << std::endl << std::endl;
    for (const cv::Point2i& pt : m_pointList)
    {
      points->push_back(sofa::defaulttype::Vec2i(pt.x, pt.y));
      std::cout << pt.x << " " << pt.y << " ";
    }
    std::cout << std::endl << std::endl;
  }
  d_points.endEdit();
}

void PointPicker2D::applyFilter(const cv::Mat& in, cv::Mat& out, bool debug)
{
  if (in.empty()) return;
  if (in.channels() == 1)
    cv::cvtColor(in, out, CV_GRAY2BGR);
  else
    in.copyTo(out);


  if (m_picker != NULL)
  {
    cv::Scalar color(0, 255, 0, 255);
    for (auto line : m_picker->epilines)
    {
      cv::line(out, cv::Point(0, -line[2] / line[1]),
               cv::Point(out.cols, -(line[2] + line[0] * out.cols) / line[1]),
               color);
    }
  }
  cv::Scalar color(0, 255, 0, 255);

  if (m_pointList.empty() && debug)
  {
    cv::putText(out, "- LeftClick: add point",
                cv::Point(15, out.rows - 55), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                CV_RGB(0, 255, 0));
    cv::putText(out, "- Ctrl + LeftClick: remove point",
                cv::Point(15, out.rows - 35), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                CV_RGB(0, 255, 0));
    cv::putText(out, "- MiddleClick: clear all points",
                cv::Point(15, out.rows - 15), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                CV_RGB(0, 255, 0));
    return;
  }
  for (const cv::Point2i& pt : m_pointList)
    cv::circle(out, pt, 3, color, 1, cv::LINE_AA);
}

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
