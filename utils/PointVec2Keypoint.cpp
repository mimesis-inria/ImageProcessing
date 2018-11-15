#include "PointVec2Keypoint.h"

namespace sofacv
{
namespace utils
{

void PointVec2Keypoint::init()
{
  addInput(&d_src);
  addOutput(&d_dst);
}

void PointVec2Keypoint::doUpdate()
{
      sofa::helper::vector<cvKeypoint>& dst = *(d_dst.beginWriteOnly());
  dst.clear();
      const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();
  for (auto pt : src)
    dst.push_back(cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}


SOFA_DECL_CLASS(PointVec2Keypoint)

int PointVec2KeypointClass =
        sofa::core::RegisterObject(
        "component to convert defaulttype::vec2i to common::cvKeypoint")
        .add<PointVec2Keypoint>();
}  // namespace utils
}  // namespace sofacv
