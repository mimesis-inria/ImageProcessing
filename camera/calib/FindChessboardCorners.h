#ifndef SOFACV_CAM_CALIB_FINDPATTERNCORNERS_H
#define SOFACV_CAM_CALIB_FINDPATTERNCORNERS_H

#include <SofaCV/SofaCV.h>
#include "camera/common/CameraSettings.h"

#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace cam
{
namespace calib
{
class SOFA_IMAGEPROCESSING_API FindPatternCorners : public ImageFilter
{
 public:
  SOFA_CLASS(FindPatternCorners, ImageFilter);

  sofa::Data < sofa::helper::vector<sofa::defaulttype::Vec2i> > d_imagePoints;
  sofa::Data<sofa::helper::OptionsGroup> d_patternType;
  sofa::Data<sofa::defaulttype::Vec2i> d_patternSize;
  sofa::Data<int> d_detectRate;
  sofa::Data<int> d_flags;
  sofa::Data<bool> d_refineCorners;

  FindPatternCorners();

  void init() override;
  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;
};

SOFA_DECL_CLASS(FindPatternCorners)

int FindPatternCornersClass =
    sofa::core::RegisterObject("detection of a calibration pattern in images")
        .add<FindPatternCorners>();

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CALIB_FINDPATTERNCORNERS_H
