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

#ifndef SOFACV_CAM_CALIB_FINDPATTERNCORNERS_H
#define SOFACV_CAM_CALIB_FINDPATTERNCORNERS_H

#include <SofaCV/SofaCV.h>
#include "camera/common/CameraSettings.h"
#include "common/ImageFilter.h"

#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace cam
{
namespace calib
{
class SOFA_IMAGEPROCESSING_API FindPatternCorners : public common::ImageFilter
{
 public:
  SOFA_CLASS(FindPatternCorners, common::ImageFilter);

  sofa::Data < sofa::helper::vector<sofa::defaulttype::Vec2i> > d_imagePoints;
  sofa::Data<sofa::helper::OptionsGroup> d_patternType;
  sofa::Data<sofa::defaulttype::Vec2i> d_patternSize;
  sofa::Data<int> d_detectRate;
  sofa::Data<int> d_flags;
  sofa::Data<bool> d_refineCorners;

  FindPatternCorners();

  void init() override;
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

SOFA_DECL_CLASS(FindPatternCorners)

int FindPatternCornersClass =
    sofa::core::RegisterObject("detection of a calibration pattern in images")
        .add<FindPatternCorners>();

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CALIB_FINDPATTERNCORNERS_H
