#include "PointVectorConverter.h"

#include <SofaORCommon/cvKeypoint.h>
//#include <SofaORCommon/CameraUtils.h>

namespace sofa
{
namespace OR
{
namespace processor
{
template <class SrcType, class DstType>
std::string PointVectorConverter<SrcType, DstType>::templateName(
		const PointVectorConverter<SrcType, DstType>* /*converter*/)
{
	return std::string(defaulttype::DataTypeName<SrcType>::name()) + "," +
				 defaulttype::DataTypeName<DstType>::name();
}

template <>
void PointVectorConverter<defaulttype::Vec2i, common::cvKeypoint>::update()
{
	helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2i>& src = d_src.getValue();

	//	if (l_functor.)
	for (auto pt : src)
		dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}

template <>
void PointVectorConverter<defaulttype::Vec2i, defaulttype::Vec2d>::update()
{
	helper::vector<defaulttype::Vec2d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2i>& src = d_src.getValue();

	//	if (l_functor.)
	for (auto pt : src)
		dst.push_back(defaulttype::Vec2d(pt.x(), pt.y()));
}

template <>
void PointVectorConverter<common::cvKeypoint, defaulttype::Vec2i>::update()
{
	helper::vector<defaulttype::Vec2i>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(defaulttype::Vec2i(kp.pt.x, kp.pt.y));
}

template <>
void PointVectorConverter<defaulttype::Vec2d, common::cvKeypoint>::update()
{
	helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2d>& src = d_src.getValue();
	for (auto pt : src)
		dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}

template <>
void PointVectorConverter<common::cvKeypoint, defaulttype::Vec2d>::update()
{
	helper::vector<defaulttype::Vec2d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(defaulttype::Vec2d(kp.pt.x, kp.pt.y));
}

template <>
void PointVectorConverter<defaulttype::Vec2d, defaulttype::Vec3d>::update()
{
	helper::vector<defaulttype::Vec3d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2d>& src = d_src.getValue();

	if (!l_cam.get())
		for (auto pt : src) dst.push_back(defaulttype::Vec3d(pt.x(), pt.y(), 0.0));
	else
		for (auto pt : src)
			dst.push_back(l_cam->get3DFrom2DPosition(pt.x(), pt.y(), d_depth.getValue()));
}
template <>
void PointVectorConverter<defaulttype::Vec2i, defaulttype::Vec3d>::update()
{
	helper::vector<defaulttype::Vec3d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2i>& src = d_src.getValue();

	if (!l_cam.get())
		for (auto pt : src) dst.push_back(defaulttype::Vec3d(pt.x(), pt.y(), 0.0));
	else
		for (auto pt : src)
			dst.push_back(l_cam->get3DFrom2DPosition(pt.x(), pt.y(), d_depth.getValue()));
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
