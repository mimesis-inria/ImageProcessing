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

#include "FeatureTriangulator.h"
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/core/ObjectFactory.h>

namespace sofaor
{
namespace processor
{
namespace cam
{
SOFA_DECL_CLASS(FeatureTriangulator)

int FeatureTriangulatorClass =
		sofa::core::RegisterObject(
        "component generating a 3D point cloud from two 2D keypoints list and "
        "their matches, and stereo camera parameters")
        .add<FeatureTriangulator>();

FeatureTriangulator::FeatureTriangulator()
		: l_cam(initLink("cam",
										 "link to CameraSettings component containing and "
										 "maintaining the camera's parameters")),
			d_keypointsL(initData(&d_keypointsL, "keypoints1",
														"input vector of left keypoints", true, true)),
      d_keypointsR(initData(&d_keypointsR, "keypoints2",
                            "input vector of right keypoints", true, true)),
			d_matches(initData(
					&d_matches, "matches",
					"input array of matches (optional if keypoints are already sorted).",
					true, true)),
      d_pointCloud(
					initData(&d_pointCloud, "positions", "output vector of 3D points"))
{
  f_listening.setValue(true);
  addAlias(&d_pointCloud, "positions_out");
}

FeatureTriangulator::~FeatureTriangulator() {}
void FeatureTriangulator::init()
{
  addInput(&d_matches);
  addInput(&d_keypointsL);
  addInput(&d_keypointsR);

  addOutput(&d_pointCloud);

	if (!l_cam.get())
		msg_error(getName() + "::init()") << "Error: No stereo camera link set. "
																				 "Please use attribute 'cam' "
																				 "to define one";

	update();
}

void FeatureTriangulator::update()
{
  std::cout << getName() << std::endl;

	//	common::matrix::sofaMat2cvMat(l_cam->getRotationMatrix(), R);
	//	common::matrix::sofaVector2cvMat(l_cam->getTranslationVector(), T);
	common::matrix::sofaMat2cvMat(l_cam->getCamera1().getProjectionMatrix(), cmL);
	common::matrix::sofaMat2cvMat(l_cam->getCamera2().getProjectionMatrix(), cmR);
	common::matrix::sofaVector2cvMat(
			l_cam->getCamera1().getDistortionCoefficients(), dvL);
	common::matrix::sofaVector2cvMat(
			l_cam->getCamera2().getDistortionCoefficients(), dvR);

	PL = cmL;
	PR = cmR;

	sofa::helper::vector<Vec3d>& pts = *(d_pointCloud.beginWriteOnly());

	const sofa::helper::vector<common::cvKeypoint>& kL = d_keypointsL.getValue();
	const sofa::helper::vector<common::cvKeypoint>& kR = d_keypointsR.getValue();
  pts.resize(kL.size());
	unsigned sizePts = 0;
  (kL.size() > kR.size()) ? (sizePts = kR.size()) : (sizePts = kL.size());

	if (d_matches.isSet())
	{
		const sofa::helper::vector<common::cvDMatch>& m = d_matches.getValue();
		for (size_t i = 0; i < m.size(); ++i)
			pts[i] = l_cam->triangulate(kL[m[i].queryIdx].pt, kR[m[i].trainIdx].pt);
	}
	else
		for (size_t i = 0; i < sizePts; ++i)
			pts[i] = l_cam->triangulate(kL[i].pt, kR[i].pt);
	d_pointCloud.endEdit();
}

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
