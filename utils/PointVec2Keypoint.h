#ifndef SOFACV_UTILS_POINTVEC2KEYPOINT_H
#define SOFACV_UTILS_POINTVEC2KEYPOINT_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace utils
{
class SOFA_IMAGEPROCESSING_API PointVec2Keypoint : public ImplicitDataEngine
{
 public:
  SOFA_CLASS(PointVec2Keypoint, ImplicitDataEngine);

  PointVec2Keypoint()
      : d_src(initData(&d_src, "points", "input vector to convert")),
        d_dst(initData(&d_dst, "points_out", "converted output vector"))
  {
  }

  virtual ~PointVec2Keypoint() override {}
  void init() override;
  void Update() override;

  // INPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_src;
  // OUTPUTS
  sofa::Data<sofa::helper::vector<cvKeypoint> > d_dst;
};

}  // namespace utils
}  // namespace sofacv

#endif  // SOFACV_UTILS_POINTVEC2KEYPOINT_H
