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

#ifndef SOFA_OR_PROCESSOR_FEATURECOLOREXTRACTOR_H
#define SOFA_OR_PROCESSOR_FEATURECOLOREXTRACTOR_H

#include <ProcessOR/common/ImageFilter.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
class FeatureColorExtractor: public ImageFilter
{
        typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;

 public:
	SOFA_CLASS(FeatureColorExtractor, ImageFilter);

 public:
	FeatureColorExtractor();
	virtual ~FeatureColorExtractor();

	void init();
    void Update();
	virtual void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug = false);

	// INPUTS
	sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypoints;

	// OUTPUTS
	sofa::Data<sofa::helper::vector<Vec3b> > d_colors;

	sofa::helper::vector<Vec3b> m_colors;
};

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FEATURECOLOREXTRACTOR_H
