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

#include "ImageProcessing/common/DataSliderMgr.h"
#include <SofaTest/Sofa_test.h>
#include <opencv2/opencv.hpp>


namespace sofa
{
struct DSM_test : public sofa::Sofa_test<>
{
//	ScalarSliderManager<bool, bool>* boolSlider;
//	ScalarSliderManager<helper::OptionsGroup, int>* boolSlider;
//	ScalarSliderManager<int, int>* boolSlider;
//	ScalarSliderManager<double, double>* boolSlider;

	DSM_test()
	{

	}


	void createBoolSlider()
	{

	}

	void SetUp()
	{
		cv::namedWindow("window");
	}

	void TearDown()
	{
		cv::destroyAllWindows();
	}
};

TEST_F(DSM_test, createBoolSlider) { this->createBoolSlider(); }

}  // namespace sofa
