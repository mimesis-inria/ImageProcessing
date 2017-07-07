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

#ifndef SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
#define SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H

#include "StereoSettings.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvDMatch.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace cam
{
/**
 * @brief The FeatureTriangulator class
 *
 * Triangulates a set of two 2D point matches to get their 3D position.
 */
class FeatureTriangulator : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<
			FeatureTriangulator, StereoSettings,
			sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
			StereoCamSettings;
	typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;
	typedef sofa::defaulttype::Vec3d Vec3d;

 public:
  SOFA_CLASS(FeatureTriangulator, common::ImplicitDataEngine);

 public:
  FeatureTriangulator();
  virtual ~FeatureTriangulator();

  void init();
  void update();

	// INPUTS
	StereoCamSettings l_cam;  ///< StereoSettings component holding the two camera
														/// settings and the fundamental matrix
	sofa::Data<sofa::helper::vector<common::cvKeypoint> >
			d_keypointsL;  ///< [INPUT] reference camera's keypoints
	sofa::Data<sofa::helper::vector<common::cvKeypoint> >
			d_keypointsR;  ///< [INPUT] second camera's keypoints
	sofa::Data<sofa::helper::vector<common::cvDMatch> >
			d_matches;  ///< [INPUT] matches between the keypoints, if not ordered

	sofa::Data<sofa::helper::vector<Vec3d> >
			d_pointCloud;  ///< [OUTPUT] triangulated 3D point cloud

 private:
  cv::Mat_<double> R;
  cv::Mat_<double> T;
  cv::Mat_<double> cmL;
  cv::Mat_<double> cmR;
  cv::Mat_<double> dvL;
  cv::Mat_<double> dvR;
  cv::Matx34d PL;
  cv::Matx34d PR;
};

}  // namespace cam
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FEATURETRIANGULATOR_H
