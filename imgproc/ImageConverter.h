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

#ifndef SOFA_OR_PROCESSOR_IMAGECONVERTER_H
#define SOFA_OR_PROCESSOR_IMAGECONVERTER_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class ImageConverter : public ImageFilter
{
 public:
	SOFA_CLASS(ImageConverter, ImageFilter);

	sofa::Data<int> d_convertTo;
	sofa::Data<double> d_scaleFactor;

	ImageConverter()
			: d_convertTo(initData(&d_convertTo, "convertTo",
															"opencv type value")),
				d_scaleFactor(initData(&d_scaleFactor, "scaleFactor", "scale Factor"))
  {
  }

  void init()
  {
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
		in.convertTo(out, d_convertTo.getValue(), d_scaleFactor.getValue());
  }
};

SOFA_DECL_CLASS(ImageConverter)

int ImageConverterClass =
		sofa::core::RegisterObject("Image type converter")
				.add<ImageConverter>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_IMAGECONVERTER_H
