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

#include "CannyFilter.h"

sofaor::processor::imgproc::CannyFilter::CannyFilter()
    : d_minThreshold(initData(&d_minThreshold, 0.0, "min",
                              "first threshold for the hysteresis procedure.")),
      d_maxThreshold(
          initData(&d_maxThreshold, 80.0, "max",
                   "second threshold for the hysteresis procedure.")),
      d_apertureSize(initData(&d_apertureSize, 3, "apertureSize",
                              "Canny's aperture size")),
      d_l2gradient(initData(&d_l2gradient, false, "L2gradient",
                            "more precision when set to true"))
{
}

void sofaor::processor::imgproc::CannyFilter::init()
{
  registerData(&d_minThreshold, 0.0, 255.0, 1.0);
  registerData(&d_maxThreshold, 0.0, 255.0, 1.0);
  registerData(&d_apertureSize, 3, 7, 2);
  registerData(&d_l2gradient);
  ImageFilter::init();
}

void sofaor::processor::imgproc::CannyFilter::applyFilter(const cv::Mat &in,
                                                          cv::Mat &out, bool)
{
  if (in.empty()) return;
  int apertureSize = d_apertureSize.getValue();
  if (apertureSize != 3 && apertureSize != 5 && apertureSize != 7)
  {
    msg_warning("CannyFilter::applyFliter()")
        << "Error: Aperture Size should be either 3, 5 or 7.";
    return;
  }
  if (d_minThreshold.getValue() < 0.0 || d_minThreshold.getValue() > 255.0 ||
      d_maxThreshold.getValue() < 0.0 || d_maxThreshold.getValue() > 255.0)
  {
    msg_warning("CannyFilter::applyFliter()")
        << "Error: Thresholds should be between 0 - 255 as we're using 8-bit "
           "grayscale images.";
    return;
  }

  cv::Mat img_gray;
  if (in.type() == CV_8UC4)
    cv::cvtColor(in, img_gray, CV_BGRA2GRAY);
  else if (in.type() == CV_8UC3)
    cv::cvtColor(in, img_gray, CV_BGR2GRAY);
  else
    img_gray = in.clone();

  cv::Canny(img_gray, img_gray, d_minThreshold.getValue(),
            d_maxThreshold.getValue(), d_apertureSize.getValue(),
            d_l2gradient.getValue());

  out = img_gray;
}
