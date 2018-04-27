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

#include "ImageRectifier.h"

namespace sofacv
{
namespace cam
{
ImageRectifier::ImageRectifier()
    : l_cam(initLink("cam",
                     "link to CameraSettings component containing and "
                     "maintaining the camera's parameters"))
{
}

void ImageRectifier::init()
{
  if (!l_cam.get())
    msg_error(getName() + "::init()") << "Error: No camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";
  ImageFilter::init();
}

void ImageRectifier::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty() || l_cam->getDistortionCoefficients().empty()) return;
  cv::Mat_<double> cam;
  matrix::sofaMat2cvMat(l_cam->getIntrinsicCameraMatrix(), cam);
  cv::undistort(in, out, cam, l_cam->getDistortionCoefficients());
}
}  // namespace cam
}  // namespace sofacv
