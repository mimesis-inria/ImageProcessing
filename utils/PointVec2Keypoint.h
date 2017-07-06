#ifndef SOFA_OR_PROCESSOR_POINTVEC2KEYPOINT_H
#define SOFA_OR_PROCESSOR_POINTVEC2KEYPOINT_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
class PointVec2Keypoint : public common::ImplicitDataEngine
{
 public:
  SOFA_CLASS(PointVec2Keypoint, common::ImplicitDataEngine);

  PointVec2Keypoint()
      : d_src(initData(&d_src, "points", "input vector to convert")),
        d_dst(initData(&d_dst, "points_out", "converted output vector"))
  {
  }

  ~PointVec2Keypoint() {}
  void init()
  {
    addInput(&d_src);
    addOutput(&d_dst);
  }

  void update()
  {
		sofa::helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
    dst.clear();
		const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();
    for (auto pt : src)
      dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
  }

  // INPUTS
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_src;
  // OUTPUTS
	sofa::Data<sofa::helper::vector<common::cvKeypoint> > d_dst;
};

SOFA_DECL_CLASS(PointVec2Keypoint)

int PointVec2KeypointClass =
		sofa::core::RegisterObject(
        "component to convert defaulttype::vec2i to common::cvKeypoint")
        .add<PointVec2Keypoint>();

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_POINTVEC2KEYPOINT_H
