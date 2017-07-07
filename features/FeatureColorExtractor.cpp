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

#include "FeatureColorExtractor.h"
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/core/ObjectFactory.h>

namespace sofaor
{
namespace processor
{
namespace features
{
SOFA_DECL_CLASS(FeatureColorExtractor)

int FeatureColorExtractorClass =
		sofa::core::RegisterObject(
				"component extracting pixel colors from two 2D keypoints")
				.add<FeatureColorExtractor>();

FeatureColorExtractor::FeatureColorExtractor()
		: d_keypoints(initData(&d_keypoints, "keypoints",
														"input vector of keypoints", true, true)),
		d_colors(initData(&d_colors, "colors",
																"output vector of rgb point color"))
{
	f_listening.setValue(true);
	addAlias(&d_colors, "colors_out");
}

FeatureColorExtractor::~FeatureColorExtractor() {}
void FeatureColorExtractor::init()
{
	addInput(&d_keypoints);

	addOutput(&d_colors);

	update();
}

void FeatureColorExtractor::update()
{
	ImageFilter::update();

	sofa::helper::vector<Vec3b>& colors = *(d_colors.beginWriteOnly());
	colors.clear();
	for (auto c : m_colors)
		colors.push_back(c);
	d_colors.endEdit();
}

void FeatureColorExtractor::applyFilter(const cv::Mat& in, cv::Mat& out, bool)
{
	out.zeros(in.rows, in.cols, in.type());
	m_colors.clear();
	const sofa::helper::vector<common::cvKeypoint>& kpts = d_keypoints.getValue();
	for (auto kp : kpts)
	{
		cv::Vec3b c = d_img.getValue().at<cv::Vec3b>(kp.pt.y, kp.pt.x);
		m_colors.push_back(Vec3b(c[0], c[1], c[2]));
		out.at<cv::Vec3b>(kp.pt.y, kp.pt.x) = c;
	}
}

}  // namespace features
}  // namespace processor
}  // namespace sofaor
