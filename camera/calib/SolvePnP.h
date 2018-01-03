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

#ifndef SOFA_OR_PROCESSOR_SOLVEPNP_H
#define SOFA_OR_PROCESSOR_SOLVEPNP_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "camera/common/CameraSettings.h"

#include <sofa/core/objectmodel/Link.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
namespace calib
{
/**
 * @brief The SolvePnP class
 *
 * Estimates the pose of a camera relative to a 3D object by using a set of
 * points on the object, and their 2D correspondances in the camera's image
 *
 * (see SolvePnP in http://docs.opencv.org/3.2.0/d9/d0c/group__calib3d.html
 * for details)
 */
class SolvePnP : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<
			SolvePnP, CameraSettings,
			sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(SolvePnP, common::ImplicitDataEngine);

    SolvePnP();

	~SolvePnP() {}
    void init();

	void update();

	CamSettings l_cam;  ///< Camera settings to update

	// INPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector2> >
			d_imagePoints;  ///< [INPUT] 2D points in the image
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector3> >
			d_objectPoints;  ///< [INPUT] 3D points on the object

	// OPTIONAL INPUT
	sofa::Data<sofa::defaulttype::Vec2i> d_imgSize;  ///< [INPUT] image size to estimate K
	sofa::Data<sofa::defaulttype::Matrix3> d_K;      ///< [INPUT] Intrinsic Guess
	sofa::Data<sofa::helper::vector<double> >
			d_distCoefs;             ///< [INPUT] Distortion coefficients guess
	sofa::Data<int> d_pnpFlags;  ///< OpenCV's PNP flags
};

}  // namespace calib
}  // namespace cam
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_SOLVEPNP_H
