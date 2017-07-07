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

#ifndef SOFA_OR_PROCESSOR_SEGMENTER2D_H
#define SOFA_OR_PROCESSOR_SEGMENTER2D_H

#include "common/ImageFilter.h"

#include <sofa/helper/SVector.h>

#include <opencv2/imgproc.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
class Segmenter2D : public ImageFilter
{
 public:
  SOFA_CLASS(Segmenter2D, ImageFilter);

	sofa::Data<std::string> d_regionLabel;

  // INPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_points;

  // OUTPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_regionPoly;
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_regionPoints;

  Segmenter2D()
			: ImageFilter(false),
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
		update();
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
		for (const sofa::defaulttype::Vec2i& pt : d_regionPoly.getValue())
    {
			polygon.push_back(cv::Point2i(pt.x(), pt.y()));
    }
		sofa::helper::vector<sofa::defaulttype::Vec2i>& points =
        *d_regionPoints.beginWriteOnly();
		points.clear();

		const sofa::helper::vector<sofa::defaulttype::Vec2i>& pts = d_points.getValue();

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
			for (const sofa::defaulttype::Vec2i& pt : d_regionPoly.getValue())
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
		sofa::core::RegisterObject("Manual segmentation component").add<Segmenter2D>();

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_SEGMENTER2D_H
