#include "PointVectorConverter.h"

#include <SofaORCommon/cvKeypoint.h>
//#include <SofaORCommon/CameraUtils.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
template <class SrcType, class DstType>
std::string PointVectorConverter<SrcType, DstType>::templateName(
		const PointVectorConverter<SrcType, DstType>* /*converter*/)
{
	return std::string(sofa::defaulttype::DataTypeName<SrcType>::name()) + "," +
				 sofa::defaulttype::DataTypeName<DstType>::name();
}

template <>
void PointVectorConverter<sofa::defaulttype::Vec2i, common::cvKeypoint>::Update()
{
	sofa::helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();

	//	if (l_functor.)
	for (auto pt : src)
		dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}

template <>
void PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec2d>::Update()
{
	sofa::helper::vector<sofa::defaulttype::Vec2d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();

	//	if (l_functor.)
	for (auto pt : src)
		dst.push_back(sofa::defaulttype::Vec2d(pt.x(), pt.y()));
}

template <>
void PointVectorConverter<common::cvKeypoint, sofa::defaulttype::Vec2i>::Update()
{
	sofa::helper::vector<sofa::defaulttype::Vec2i>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(sofa::defaulttype::Vec2i(kp.pt.x, kp.pt.y));
}

template <>
void PointVectorConverter<sofa::defaulttype::Vec2d, common::cvKeypoint>::Update()
{
	sofa::helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<sofa::defaulttype::Vec2d>& src = d_src.getValue();
	for (auto pt : src)
		dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}

template <>
void PointVectorConverter<common::cvKeypoint, sofa::defaulttype::Vec2d>::Update()
{
	sofa::helper::vector<sofa::defaulttype::Vec2d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(sofa::defaulttype::Vec2d(kp.pt.x, kp.pt.y));
}

template <>
void PointVectorConverter<sofa::defaulttype::Vec2d, sofa::defaulttype::Vec3d>::Update()
{
	sofa::helper::vector<sofa::defaulttype::Vec3d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<sofa::defaulttype::Vec2d>& src = d_src.getValue();

	if (!l_cam.get())
		for (auto pt : src) dst.push_back(sofa::defaulttype::Vec3d(pt.x(), pt.y(), 0.0));
	else
		for (auto pt : src)
			dst.push_back(l_cam->get3DFrom2DPosition(pt.x(), pt.y(), d_depth.getValue()));
}
template <>
void PointVectorConverter<sofa::defaulttype::Vec2i, sofa::defaulttype::Vec3d>::Update()
{
	sofa::helper::vector<sofa::defaulttype::Vec3d>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const sofa::helper::vector<sofa::defaulttype::Vec2i>& src = d_src.getValue();

	if (!l_cam.get())
		for (auto pt : src) dst.push_back(sofa::defaulttype::Vec3d(pt.x(), pt.y(), 0.0));
	else
		for (auto pt : src)
			dst.push_back(l_cam->get3DFrom2DPosition(pt.x(), pt.y(), d_depth.getValue()));
}

}  // namespace utils
}  // namespace processor
}  // namespace sofaor
