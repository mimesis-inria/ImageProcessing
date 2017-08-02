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

#ifndef SOFA_OR_PROCESSOR_OPTICALFLOW_H
#define SOFA_OR_PROCESSOR_OPTICALFLOW_H

#include "common/ImageFilter.h"

#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/vector.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/video.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
class OpticalFlow : public ImageFilter
{
 public:
  SOFA_CLASS(OpticalFlow, ImageFilter);

  sofa::Data<sofa::defaulttype::Vec2i> d_winSize;
  sofa::Data<int> d_maxLevel;
  sofa::Data<int> d_criteria_type;
  sofa::Data<int> d_maxCount;
  sofa::Data<double> d_epsilon;
  sofa::Data<int> d_flags;
  sofa::Data<double> d_minEigThresh;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2d> > d_points_in;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2d> > d_points_out;
  sofa::Data<sofa::helper::vector<uchar> > d_status_out;
  sofa::Data<sofa::helper::vector<float> > d_error_out;
  sofa::Data<bool> d_startTracking;

  std::vector<cv::Point2f> m_pts_in;
  std::vector<cv::Point2f> m_pts_out;

  OpticalFlow()
      : d_winSize(initData(&d_winSize, sofa::defaulttype::Vec2i(21, 21),
                           "win_size", "")),
        d_maxLevel(initData(&d_maxLevel, 3, "max_level", "")),
        d_criteria_type(initData(&d_criteria_type,
                                 CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
                                 "crit_type", "")),
        d_maxCount(initData(&d_maxCount, 30, "max_count", "")),
        d_epsilon(initData(&d_epsilon, 0.01, "epsilon", "")),
        d_flags(initData(&d_flags, 0, "flags", "")),
        d_minEigThresh(initData(&d_minEigThresh, 1e-4, "eigen_threshold", "")),
        d_points_in(
            initData(&d_points_in, "points", "set of input points to track")),
        d_points_out(initData(&d_points_out, "points_out",
                              "set of output points to track")),
        d_startTracking(initData(&d_startTracking, false, "start",
                                 "set to true to stop reading from input "
                                 "vector 'points' and start performing the "
                                 "optical flow"))
  {
  }

  void init()
  {
    registerData(&d_maxLevel, 0, 10, 1);
    registerData(&d_maxCount, 0, 100, 1);
    registerData(&d_epsilon, 0.0, 0.2, 0.001);
    registerData(&d_minEigThresh, 0.001, 0.1, 0.001);

    addInput(&d_points_in);
    addOutput(&d_points_out);
    ImageFilter::init();
    update();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    cv::Mat gray;
    if (in.empty()) return;

    if (!d_startTracking.getValue() || m_pts_in.empty())
    {
      // Keep things well initialized:
      if (!m_prev.empty())  // m_prev should stay empty
        m_prev.zeros(m_prev.rows, m_prev.cols, m_prev.type());

      // set prev_points to the current input
      if (d_points_in.getValue().empty()) return;
      m_pts_in.reserve(d_points_in.getValue().size());
      m_pts_in.clear();
      for (const sofa::defaulttype::Vec2d& pt : d_points_in.getValue())
        m_pts_in.push_back(cv::Point2f(pt.x(), pt.y()));

      // copy in in out
      in.copyTo(out);
      d_points_out.setValue(d_points_in.getValue());
      return;
    }

    // IF THE TRACKER HAS BEEN STARTED:
    if (in.type() != CV_8UC1)
      cv::cvtColor(in, gray, CV_BGRA2GRAY);
    else
      gray = in;

    // If it's the first step of the optical flow:
    if (m_prev.empty())
    {
      // we then want to initialize some of the prev values:
      m_prev = gray.clone();
      return;
    }
    m_pts_out = m_pts_in;

    if (m_pts_in.empty() || m_prev.empty() || gray.empty())
    {
      msg_error(getName() + "::applyFilter()")
          << "something is wrong: please check your input frames and / "
             "or input points";
      return;
    }
    std::vector<uchar> status = d_status_out.getValue();
    std::vector<float> error = d_error_out.getValue();

    cv::TermCriteria tc =
        cv::TermCriteria(d_criteria_type.getValue(), d_maxCount.getValue(),
                         d_epsilon.getValue());
    cv::Size winSize(d_winSize.getValue().x(), d_winSize.getValue().y());

    cv::calcOpticalFlowPyrLK(m_prev, gray, m_pts_in, m_pts_out, status, error,
                             winSize, d_maxLevel.getValue(), tc,
                             d_flags.getValue(), d_minEigThresh.getValue());

    sofa::helper::vector<sofa::defaulttype::Vec2d>* points_out =
        d_points_out.beginEdit();
    sofa::helper::vector<sofa::defaulttype::Vec2d>* points_in =
        d_points_in.beginEdit();
    points_out->clear();
    points_in->clear();
    for (size_t i = 0; i < m_pts_out.size(); ++i)
    {
      cv::Point2f ptPrev, ptNext;
      ptPrev = m_pts_in[i];
      ptNext = m_pts_out[i];
      points_out->push_back(sofa::defaulttype::Vec2d(ptNext.x, ptNext.y));
      points_in->push_back(sofa::defaulttype::Vec2d(ptPrev.x, ptPrev.y));
    }
    d_status_out.setValue(status);
    d_error_out.setValue(error);

    m_pts_in = m_pts_out;
    d_points_out.endEdit();
    d_points_in.endEdit();
    m_prev = gray.clone();

    // copy in in out
    in.copyTo(out);
    if (d_displayDebugWindow.getValue())
    {
      for (size_t i = 0; i < m_pts_out.size(); ++i)
      {
        if (!status[i])
          cv::circle(out, m_pts_out[i], 3, cv::Scalar(0, 0, 255), 1,
                     cv::LINE_AA);
        else
          cv::circle(out, m_pts_out[i], 3, cv::Scalar(0, 255, 0), 1,
                     cv::LINE_AA);
      }
    }
  }

 private:
  cv::Mat m_prev;
};

SOFA_DECL_CLASS(OpticalFlow)

int OpticalFlowClass =
    sofa::core::RegisterObject("Optical flow filters from OpenCV")
        .add<OpticalFlow>();

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_OPTICALFLOW_H
