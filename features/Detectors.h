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

#ifndef SOFA_OR_PROCESSOR_DETECTORS_H
#define SOFA_OR_PROCESSOR_DETECTORS_H

#include "common/ImageFilter.h"

#include <SofaORCommon/cvMat.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vector.h>

#include <opencv2/xfeatures2d.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
struct BaseDetector
{
  virtual ~BaseDetector();

  virtual void toggleVisible(bool) = 0;

  virtual void init() = 0;

  virtual void registerData(ImageFilter* parent) = 0;

  virtual void detect(const common::cvMat&, const common::cvMat&,
                      std::vector<cv::KeyPoint>&);
  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&);
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&);

 protected:
  cv::Ptr<cv::Feature2D> m_detector;
};

struct SimpleBlobDetector : BaseDetector
{
	SimpleBlobDetector(sofa::core::objectmodel::BaseObject* c);
	void toggleVisible(bool);
	void init();

	virtual void registerData(ImageFilter* parent)
	{
		parent->registerData(&minThreshold, 0, 255, 1);
		parent->registerData(&maxThreshold, 0, 255, 1);
		parent->registerData(&filterByArea);
		parent->registerData(&minArea, 0, 3000, 2);
		parent->registerData(&filterByCircularity);
		parent->registerData(&minCircularity, 0.0, 1.0, 0.001);
		parent->registerData(&filterByConvexity);
		parent->registerData(&minConvexity, 0.0, 1.0, 0.01);
		parent->registerData(&filterByInertia);
		parent->registerData(&minInertiaRatio, 0.0, 1.0, 0.01);
	}

	virtual void detect(const common::cvMat& img, const common::cvMat& mask,
											std::vector<cv::KeyPoint>& keypoints)
	{
		// Setup SimpleBlobDetector parameters.
		cv::SimpleBlobDetector::Params params;

		// Change thresholds
		params.minThreshold = minThreshold.getValue();
		params.maxThreshold = maxThreshold.getValue();

		// Filter by Area.
		params.filterByArea = filterByArea.getValue();
		params.minArea = minArea.getValue();

		// Filter by Circularity
		params.filterByCircularity = filterByCircularity.getValue();
		params.minCircularity = minCircularity.getValue();

		// Filter by Convexity
		params.filterByConvexity = filterByConvexity.getValue();
		params.minConvexity = minConvexity.getValue();

		// Filter by Inertia
		params.filterByInertia = filterByInertia.getValue();
		params.minInertiaRatio = minInertiaRatio.getValue();

#if CV_MAJOR_VERSION < 3  // If you are using OpenCV 2

		// Set up detector with params
		cv::SimpleBlobDetector detector(params);

		// You can use the detector this way
		detector.detect(img, keypoints);

#else

		// Set up detector with params
		cv::Ptr<cv::SimpleBlobDetector> detector =
				cv::SimpleBlobDetector::create(params);

		// SimpleBlobDetector::create creates a smart pointer.
		// So you need to use arrow ( ->) instead of dot ( . )
		detector->detect(img, keypoints, mask);

#endif
	}

	virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
											 common::cvMat&)
	{
		msg_error("SimpleBlobDetector::compute()")
				<< "SimpleBlob is detectOnly. descriptors won't be computed.";
	}
	virtual void detectAndCompute(const common::cvMat& img,
																const common::cvMat& mask,
																std::vector<cv::KeyPoint>& kpts, common::cvMat&)
	{
		msg_warning("SimpleBlobDetector::detectAndCompute()")
				<< "SimpleBlob is detectOnly. descriptors won't be computed.";
		detect(img, mask, kpts);
	}

	sofa::Data<int> minThreshold;
	sofa::Data<int> maxThreshold;
	sofa::Data<bool> filterByArea;
	sofa::Data<int> minArea;
	sofa::Data<bool> filterByCircularity;
	sofa::Data<double> minCircularity;
	sofa::Data<bool> filterByConvexity;
	sofa::Data<double> minConvexity;
	sofa::Data<bool> filterByInertia;
	sofa::Data<double> minInertiaRatio;
};

struct ShiTomasiDetector : BaseDetector
{
	ShiTomasiDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter* parent)
  {
    parent->registerData(&maxCorners, 0, 255, 1);
    parent->registerData(&qualityLevel, 0.0, 1.0, 0.01);
    parent->registerData(&minDistance, 0, 20, 1);
    parent->registerData(&blockSize, 0, 20, 1);
  }

	virtual void detect(const common::cvMat&, const common::cvMat&,
											std::vector<cv::KeyPoint>&)
  {
    msg_error("ShiTomasiDetector::detect()") << "Not Implemented Yet";
  }

  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_error("FASTDetector::compute()")
        << "FAST is detectOnly. descriptors won't be computed.";
  }
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&)
  {
    msg_warning("FASTDetector::detectAndCompute()")
        << "FAST is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
  }

	sofa::Data<int> maxCorners;
	sofa::Data<double> qualityLevel;
	sofa::Data<int> minDistance;
	sofa::Data<int> blockSize;
};

struct FASTDetector : BaseDetector
{
	FASTDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter* parent)
  {
    parent->registerData(&threshold, 0, 255, 1);
    parent->registerData(&nonmaxsuppression);
    parent->registerData(&type);
  }
  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_error("FASTDetector::compute()")
        << "FAST is detectOnly. descriptors won't be computed.";
  }
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&)
  {
    msg_warning("FASTDetector::detectAndCompute()")
        << "FAST is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
  }

	sofa::Data<int> threshold;
	sofa::Data<bool> nonmaxsuppression;
	sofa::Data<sofa::helper::OptionsGroup> type;
};
struct MSERDetector : BaseDetector
{
	MSERDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter* parent)
  {
    parent->registerData(&delta, 0, 10, 1);
    parent->registerData(&minArea, 0, 255, 1);
    parent->registerData(&maxArea, 10000, 20000, 200);

    parent->registerData(&maxVariation, 0.0f, 1.0f, 0.01f);
    parent->registerData(&minDiversity, 0.0f, 1.0f, 0.01f);
    parent->registerData(&maxEvolution, 0, 400, 1);
    parent->registerData(&areaThreshold, 1.0, 2.0, 0.01);
    parent->registerData(&minMargin, 0.0, 0.1, 0.0001);
    parent->registerData(&edgeBlurSize, 0, 10, 1);
  }

  virtual void compute(const common::cvMat&, std::vector<cv::KeyPoint>&,
                       common::cvMat&)
  {
    msg_error("MSERDetector::detectAndCompute()")
        << "MSER is detectOnly. descriptors won't be computed.";
  }
  virtual void detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& kpts, common::cvMat&)
  {
    msg_warning("MSERDetector::detectAndCompute()")
        << "MSER is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
  }

	sofa::Data<int> delta;
	sofa::Data<int> minArea;
	sofa::Data<int> maxArea;
	sofa::Data<float> maxVariation;
	sofa::Data<float> minDiversity;
	sofa::Data<int> maxEvolution;
	sofa::Data<double> areaThreshold;
	sofa::Data<double> minMargin;
	sofa::Data<int> edgeBlurSize;
};
struct ORBDetector : BaseDetector
{
	ORBDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();

  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

	sofa::Data<int> nFeatures;
	sofa::Data<float> scaleFactor;
	sofa::Data<int> nLevels;
	sofa::Data<int> edgeThreshold;
	sofa::Data<int> firstLevel;
	sofa::Data<int> WTA_K;
	sofa::Data<sofa::helper::OptionsGroup> scoreType;
	sofa::Data<int> patchSize;
	sofa::Data<int> fastThreshold;
};
struct BRISKDetector : BaseDetector
{
	BRISKDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

	sofa::Data<int> threshold;
	sofa::Data<int> octaves;
	sofa::Data<float> npatternScale;
};
struct KAZEDetector : BaseDetector
{
	KAZEDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

	sofa::Data<bool> extended;
	sofa::Data<bool> upright;
	sofa::Data<float> threshold;
	sofa::Data<int> octaves;
	sofa::Data<int> sublevels;
	sofa::Data<sofa::helper::OptionsGroup> diffusivity;
};
struct AKAZEDetector : BaseDetector
{
	AKAZEDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

	sofa::Data<sofa::helper::OptionsGroup> descriptorType;
	sofa::Data<int> descriptorSize;
	sofa::Data<int> descriptorChannels;
	sofa::Data<float> threshold;
	sofa::Data<int> octaves;
	sofa::Data<int> sublevels;
	sofa::Data<sofa::helper::OptionsGroup> diffusivity;
};

#ifdef SOFAOR_OPENCV_CONTRIB_ENABLED

struct BRIEFDetector : BaseDetector
{
	BRIEFDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

  void detect(const common::cvMat&, const common::cvMat&,
              std::vector<cv::KeyPoint>&)
  {
    msg_error("BRIEFDetector::detect()")
        << "BRIEF is computeOnly. keypoints must be provided.";
  }
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&)
  {
    msg_error("BRIEFDetector::detectAndCompute()")
        << "BRIEF is computeOnly. Please provide keypoints and set "
           "DetectMode to COMPUTE_ONLY";
  }

	sofa::Data<int> bytes;
	sofa::Data<bool> use_orientation;
};

struct SIFTDetector : BaseDetector
{
	SIFTDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

	sofa::Data<int> nFeatures;
	sofa::Data<int> nOctaveLayers;
	sofa::Data<double> contrastThreshold;
	sofa::Data<double> edgeThreshold;
	sofa::Data<double> sigma;
};

struct SURFDetector : BaseDetector
{
	SURFDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

	sofa::Data<double> threshold;
	sofa::Data<int> nOctaves;
	sofa::Data<int> nOctaveLayers;
	sofa::Data<bool> extended;
	sofa::Data<bool> upright;
};

struct DAISYDetector : BaseDetector
{
	DAISYDetector(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  virtual void registerData(ImageFilter*)
  {
    // TODO: find optimal range of values
  }

  void detect(const common::cvMat&, const common::cvMat&,
              std::vector<cv::KeyPoint>&)
  {
    msg_error("DAISYDetector::detect()")
        << "DAISY is computeOnly. keypoints must be provided.";
  }
  virtual void detectAndCompute(const common::cvMat&, const common::cvMat&,
                                std::vector<cv::KeyPoint>&, common::cvMat&)
  {
    msg_error("DAISYDetector::detectAndCompute()")
        << "DAISY is computeOnly. Please provide keypoints and set "
           "DetectMode to COMPUTE_ONLY";
  }

	sofa::Data<float> radius;
	sofa::Data<int> q_radius;
	sofa::Data<int> q_theta;
	sofa::Data<int> q_hist;
	sofa::Data<sofa::helper::OptionsGroup> norm;
	sofa::Data<common::cvMat> H;
	sofa::Data<bool> interpolation;
	sofa::Data<bool> use_orientation;
};

#endif  // SOFAOR_OPENCV_CONTRIB_ENABLED

}  // namespace features
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_DETECTORS_H
