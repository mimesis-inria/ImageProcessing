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
        d_points(initData(&d_points, "points_out",
                          "output vector of 2D points picked in the image"))
  {
  }

  void init()
  {
    addOutput(&d_points);
    ImageFilter::activateMouseCallback();
    setMouseState(&PointPicker2D::freeMove);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    in.copyTo(out);
    cv::Scalar color(255, 255, 255, 50);
    std::vector<cv::Point2i> pts;
    const helper::vector<defaulttype::Vec2i>& points = d_points.getValue();

    if (points.empty()) return;
    for (const defaulttype::Vec2i& pt : points)
      cv::circle(out, cv::Point2i(pt.x(), pt.y()), 3, color, 2, cv::LINE_AA);
    cv::addWeighted(in, 0.8, out, 0.2, 0.0, out);
  }

  // Mouse controls
  typedef void (PointPicker2D::*StateFunction)(int, int, int, int);
  void freeMove(int event, int x, int y,
                int flags);  // mouse is moving, buttons are not pressed
  void capture(int event, int x, int y,
               int flags);  // left is down, capturing motion

  StateFunction m_activeState;
  void setMouseState(StateFunction f) { m_activeState = f; }
  void mouseCallback(int event, int x, int y, int flags);
};

SOFA_DECL_CLASS(PointPicker2D)

int PointPicker2DClass =
    core::RegisterObject("Manual 2D image point picker component")
        .add<PointPicker2D>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTPICKER2D_H
