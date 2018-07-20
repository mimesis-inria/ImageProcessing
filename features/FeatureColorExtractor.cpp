#include "FeatureColorExtractor.h"
#include <SofaCV/SofaCV.h>

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace features
{
SOFA_DECL_CLASS(FeatureColorExtractor)

int FeatureColorExtractorClass =
        sofa::core::RegisterObject(
            "component extracting pixel colors from two 2D keypoints")
        .add<FeatureColorExtractor>();

FeatureColorExtractor::FeatureColorExtractor()
    : d_keypoints(initData(&d_keypoints, "keypoints",
                           "input vector of keypoints", true, true)),
      d_colors(initData(&d_colors, "colors",
                        "output vector of rgb point color"))
{
    f_listening.setValue(true);
    addAlias(&d_colors, "colors_out");
}

FeatureColorExtractor::~FeatureColorExtractor() {}
void FeatureColorExtractor::init()
{
    addInput(&d_keypoints);

    addOutput(&d_colors);

    common::ImageFilter::init();
}

void FeatureColorExtractor::Update()
{
    common::ImageFilter::Update();

    sofa::helper::vector<Vec3b>& colors = *(d_colors.beginWriteOnly());
    colors.clear();
    for (auto c : m_colors)
        colors.push_back(c);
    d_colors.endEdit();
}

void FeatureColorExtractor::applyFilter(const cv::Mat& in, cv::Mat& out, bool)
{
    out.zeros(in.rows, in.cols, in.type());
    m_colors.clear();
    const sofa::helper::vector<cvKeypoint>& kpts = d_keypoints.getValue();
    for (auto kp : kpts)
    {
        cv::Vec3b c = d_img.getValue().at<cv::Vec3b>(kp.pt.y, kp.pt.x);
        m_colors.push_back(Vec3b(c[0], c[1], c[2]));
        out.at<cv::Vec3b>(kp.pt.y, kp.pt.x) = c;
    }
}

}  // namespace features
}  // namespace sofacv
