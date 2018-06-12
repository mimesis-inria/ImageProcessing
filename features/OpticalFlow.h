#ifndef SOFACV_FEATURES_OPTICALFLOW_H
#define SOFACV_FEATURES_OPTICALFLOW_H

#include "common/ImageFilter.h"

#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/vector.h>

namespace sofacv
{
namespace features
{
class SOFA_IMAGEPROCESSING_API OpticalFlow : public common::ImageFilter
{
 public:
  SOFA_CLASS(OpticalFlow, common::ImageFilter);

  sofa::Data<sofa::defaulttype::Vec2i> d_winSize;
  sofa::Data<int> d_maxLevel;
  sofa::Data<int> d_criteria_type;
  sofa::Data<int> d_maxCount;
  sofa::Data<double> d_epsilon;
  sofa::Data<int> d_flags;
  sofa::Data<double> d_minEigThresh;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2d> > d_points_in;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2d> > d_points_out;
  sofa::Data<sofa::helper::vector<uchar> > d_status_out;
  sofa::Data<sofa::helper::vector<float> > d_error_out;
  sofa::Data<bool> d_startTracking;

  std::vector<cv::Point2f> m_pts_in;
  std::vector<cv::Point2f> m_pts_out;

  OpticalFlow();

  void init();

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool);

 private:
  cv::Mat m_prev;
};

SOFA_DECL_CLASS(OpticalFlow)

int OpticalFlowClass =
    sofa::core::RegisterObject("Optical flow filters from OpenCV")
        .add<OpticalFlow>();

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_OPTICALFLOW_H
