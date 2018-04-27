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

#ifndef SOFACV_CAM_IMAGERECTIFIER_H
#define SOFACV_CAM_IMAGERECTIFIER_H

#include "CameraSettings.h"
#include "common/ImageFilter.h"
#include "SofaCV/SofaCV.h"

#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace cam
{
/**
 * @brief The ImageRectifier class
 *
 * Rectifies a given image frame using the linked CameraSettings parameters
 */
class SOFA_IMAGEPROCESSING_API ImageRectifier : public common::ImageFilter
{
  typedef sofa::core::objectmodel::SingleLink<
      ImageRectifier, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(ImageRectifier, common::ImageFilter);

  ImageRectifier();

  void init();
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

  CamSettings l_cam;  ///< linked CameraSettings component
};

SOFA_DECL_CLASS(ImageRectifier)

int ImageRectifierClass =
    sofa::core::RegisterObject("Image undistortion").add<ImageRectifier>();

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_IMAGERECTIFIER_H
