#ifndef SOFA_OR_PROCESSOR_SEGMENTER2D_H
#define SOFA_OR_PROCESSOR_SEGMENTER2D_H

#include "core/ImageFilter.h"

#include <sofa/helper/SVector.h>

#include <opencv2/imgproc.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class Segmenter2D : public ImageFilter
{
 public:
  SOFA_CLASS(Segmenter2D, ImageFilter);

	Data<std::string> d_regionLabel;

  // INPUTS
	Data<helper::vector<defaulttype::Vec2f> > d_points;

  // OUTPUTS
	Data<helper::vector<defaulttype::Vec2i> > d_regionPoly;
	Data<helper::vector<defaulttype::Vec2f> > d_regionPoints;

  Segmenter2D()
			: ImageFilter(0),
				d_regionLabel(initData(&d_regionLabel, "label",
															 "label for the segmented region")),
				d_points(initData(&d_points, "points", "input vector keypoints")),
				d_regionPoly(initData(&d_regionPoly, "poly", "optional input polygon")),
				d_regionPoints(initData(&d_regionPoints, "points_out",
																"output vector of points fitting in the poly"))
  {
    addAlias(&d_regionPoly, "poly_out");
	}

  void init()
  {
		addInput(&d_points);
		addInput(&d_regionPoly);
		addOutput(&d_regionPoints);
    ImageFilter::activateMouseCallback();
    setMouseState(&Segmenter2D::freeMove);
    ImageFilter::init();
  }

  void update()
  {
    ImageFilter::update();

		if (d_regionPoly.getValue().size() <= 2)
		{
			d_regionPoints.setValue(d_points.getValue());
			return;
		}

		std::vector<cv::Point2i> polygon;
		for (const defaulttype::Vec2i& pt : d_regionPoly.getValue())
    {
			polygon.push_back(cv::Point2i(pt.x(), pt.y()));
    }
		helper::vector<defaulttype::Vec2f>& points =
        *d_regionPoints.beginWriteOnly();
		points.clear();

		const helper::vector<defaulttype::Vec2f>& pts = d_points.getValue();

		for (auto point : pts)
		{
			cv::Point2f pt(point.x(), point.y());
			if (cv::pointPolygonTest(polygon, pt, false) > 0) points.push_back(point);
		}
    d_regionPoints.endEdit();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    in.copyTo(out);
		std::vector<std::vector<cv::Point2i> > polygon(1);
		if (m_activeState == &Segmenter2D::freeMove)
    {
      if (d_regionPoly.getValue().empty()) return;
			for (const defaulttype::Vec2i& pt : d_regionPoly.getValue())
      {
				polygon[0].push_back(cv::Point2i(pt.x(), pt.y()));
      }
    }
    else
    {
			if (m_poly.size() < 2) return;
			polygon[0] = std::vector<cv::Point2i>(m_poly.begin(), m_poly.end());
    }
    cv::Scalar color(255, 255, 255, 50);
		cv::fillPoly(out, polygon, color);
    cv::addWeighted(in, 0.8, out, 0.2, 0.0, out);
  }

  // Mouse controls
  typedef void (Segmenter2D::*StateFunction)(int, int, int, int);
  void freeMove(int event, int x, int y,
                int flags);  // mouse is moving, buttons are not pressed
  void capture(int event, int x, int y,
               int flags);  // left is down, capturing motion
  void capturePaused(int event, int x, int y,
                     int flags);  // left is down, capturing motion
  void stopping(int event, int x, int y, int flags);  // right is down, stopping

  StateFunction m_activeState;
  void setMouseState(StateFunction f) { m_activeState = f; }
  void mouseCallback(int event, int x, int y, int flags);

  std::list<cv::Point2i> m_poly;
};

SOFA_DECL_CLASS(Segmenter2D)

int Segmenter2DClass =
    core::RegisterObject("Manual segmentation component").add<Segmenter2D>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_SEGMENTER2D_H
