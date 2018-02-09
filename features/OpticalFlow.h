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

  OpticalFlow();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

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
