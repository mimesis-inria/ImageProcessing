#ifndef SOFA_OR_PROCESSOR_POINTPICKER2D_H
#define SOFA_OR_PROCESSOR_POINTPICKER2D_H

#include "core/ImageFilter.h"

#include <sofa/helper/SVector.h>

#include <opencv2/imgproc.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class PointPicker2D : public ImageFilter
{
 public:
  SOFA_CLASS(PointPicker2D, ImageFilter);

  // OUTPUT
  Data<helper::vector<defaulttype::Vec2i> > d_points;

  PointPicker2D()
      : ImageFilter(false),
        d_points(initData(&d_points, "points",
                          "output vector of 2D points picked in the image",
                          true, false))
  {
      addAlias(&d_points, "points_out");
  }

  void init()
  {
    addOutput(&d_points);
    ImageFilter::activateMouseCallback();
    setMouseState(&PointPicker2D::freeMove);
    ImageFilter::init();
  }

  void update()
  {
    ImageFilter::update();
    helper::vector<defaulttype::Vec2i>* points = d_points.beginWriteOnly();
    points->clear();
    if (!m_pointList.empty())
      for (const cv::Point2i& pt : m_pointList)
        points->push_back(defaulttype::Vec2i(pt.x, pt.y));
    d_points.endEdit();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    in.copyTo(out);
    cv::Scalar color(0, 255, 0, 255);

    if (m_pointList.empty()) return;
    for (const cv::Point2i& pt : m_pointList)
      cv::circle(out, pt, 3, color, 1, cv::LINE_AA);
  }

 protected:
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
  std::list<cv::Point2i> m_pointList;
};

SOFA_DECL_CLASS(PointPicker2D)

int PointPicker2DClass =
    core::RegisterObject("Manual 2D image point picker component")
        .add<PointPicker2D>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTPICKER2D_H
