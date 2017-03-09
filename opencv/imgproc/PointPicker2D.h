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

	// INPUTS
	Data<defaulttype::Matrix3> d_F;
	Data<int> d_whichImage;
	Data<std::string> d_getEpilinesFrom;
	// OUTPUTS
	Data<helper::vector<defaulttype::Vec2i> > d_points;
	helper::vector<defaulttype::Vec3f> epilines;

  PointPicker2D()
      : ImageFilter(false),
				d_F(initData(
						&d_F, "F",
						"optional input fundamental matrix to compute epipolar lines")),
				d_whichImage(initData(&d_whichImage, "whichImage",
															"optional input integer to define if it's Left "
															"(1) or Right(2) image, to compute epipolar "
															"lines")),
				d_getEpilinesFrom(initData(
						&d_getEpilinesFrom, "getEpilinesFrom",
						"optional input component from which to look for epipolar lines")),
				d_points(initData(&d_points, "points",
                          "output vector of 2D points picked in the image",
													true, true))
	{
		addAlias(&d_points, "points_out");
	}

  void init()
  {
		addInput(&d_F);
		addInput(&d_whichImage);
		addOutput(&d_points);
		ImageFilter::activateMouseCallback();
    setMouseState(&PointPicker2D::freeMove);
    ImageFilter::init();
		m_picker =
				this->getContext()->get<PointPicker2D>(d_getEpilinesFrom.getValue());
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

    if (m_pointList.empty()) return;
    for (const cv::Point2i& pt : m_pointList)
      cv::circle(out, pt, 3, color, 1, cv::LINE_AA);
  }

	void computeEpipolarLines();

 protected:
	PointPicker2D* m_picker;

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
	std::list<cv::Point2f> m_pointList;
};

SOFA_DECL_CLASS(PointPicker2D)

int PointPicker2DClass =
    core::RegisterObject("Manual 2D image point picker component")
        .add<PointPicker2D>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTPICKER2D_H
