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

#ifndef SOFA_OR_PROCESSOR_FINDPATTERNCORNERS_H
#define SOFA_OR_PROCESSOR_FINDPATTERNCORNERS_H

#include <SofaORCommon/cvMatUtils.h>
#include "ProcessOR/camera/common/CameraSettings.h"
#include "ProcessOR/common/ImageFilter.h"

#include <opencv2/imgproc.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
class FindPatternCorners : public ImageFilter
{
 public:
	SOFA_CLASS(FindPatternCorners, ImageFilter);

	Data < helper::vector<defaulttype::Vec2i> d_imagePoints;
	Data<helper::OptionsGroup> d_patternType;
	Data<defaulttype::Vec2i> d_patternSize;
	Data<int> d_detectRate;
	Data<int> d_flags;
	Data<bool> d_refineCorners;

    FindPatternCorners();

    void init();
    void applyFilter(const cv::Mat& in, cv::Mat& out, bool);
};

SOFA_DECL_CLASS(FindPatternCorners)

int FindPatternCornersClass =
		core::RegisterObject("detection of a calibration pattern in images")
				.add<FindPatternCorners>();

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FINDPATTERNCORNERS_H
