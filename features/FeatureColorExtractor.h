#ifndef SOFA_OR_PROCESSOR_FEATURECOLOREXTRACTOR_H
#define SOFA_OR_PROCESSOR_FEATURECOLOREXTRACTOR_H

#include <ProcessOR/common/ImageFilter.h>
#include <SofaORCommon/cvKeypoint.h>
#include <SofaORCommon/cvMat.h>
#include <SofaORCommon/cvMatUtils.h>

#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
class FeatureColorExtractor: public ImageFilter
{
        typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;

 public:
	SOFA_CLASS(FeatureColorExtractor, ImageFilter);

 public:
	FeatureColorExtractor();
	virtual ~FeatureColorExtractor();

	void init();
	void update();
	virtual void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug = false);

	// INPUTS
	sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_keypoints;

	// OUTPUTS
	sofa::Data<sofa::helper::vector<Vec3b> > d_colors;

	sofa::helper::vector<Vec3b> m_colors;
};

}  // namespace features
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FEATURECOLOREXTRACTOR_H
