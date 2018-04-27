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

#ifndef SOFACV_FEATURES_FEATUREDETECTOR_H
#define SOFACV_FEATURES_FEATUREDETECTOR_H

#include "Detectors.h"
#include "common/ImageFilter.h"

#include <SofaCV/SofaCV.h>

#include <sofa/core/DataTracker.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace features
{
class SOFA_IMAGEPROCESSING_API FeatureDetector : public common::ImageFilter
{
  enum DetectorMode
  {
    DETECT_ONLY,
    COMPUTE_ONLY,
    DETECT_AND_COMPUTE
  };

  enum DetectorType
  {
    FAST = 0,
    MSER = 1,
    ORB = 2,
    BRISK = 3,
    KAZE = 4,
    AKAZE = 5,
    BLOB = 7,
#ifdef SOFAOR_OPENCV_CONTRIB_ENABLED
    BRIEF = 6,
    SIFT = 8,
    SURF = 9,
    DAISY = 10,
#endif // SOFAOR_OPENCV_CONTRIB_ENABLED
    DetectorType_COUNT
  };

 public:
  SOFA_CLASS(FeatureDetector, common::ImageFilter);

 public:
  FeatureDetector();
  virtual ~FeatureDetector() override;

  virtual void init() override;
  virtual void reinit() override;
  virtual void Update() override;
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug) override;

	sofa::Data<sofa::helper::OptionsGroup> d_detectMode;
    sofa::Data<cvMat> d_mask;
	sofa::Data<sofa::helper::OptionsGroup> d_detectorType;
    sofa::Data<sofa::helper::vector<cvKeypoint> > d_keypoints;
    sofa::Data<cvMat> d_descriptors;

 protected:
  void detectTypeChanged();
  void detectModeChanged();

 private:
  BaseDetector* m_detectors[DetectorType_COUNT];

  std::vector<cv::KeyPoint> _v;
  cvMat _d;
};

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_FEATUREDETECTOR_H
