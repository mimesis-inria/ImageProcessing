#include "PointVectorConverter.h"

#include <SofaORCommon/cvKeypoint.h>

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
void PointVectorConverter<common::cvKeypoint, defaulttype::Vec2i>::update()
{
	helper::vector<defaulttype::Vec2i>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(defaulttype::Vec2i(kp.pt.x, kp.pt.y));
}

template <>
void PointVectorConverter<defaulttype::Vec2f, common::cvKeypoint>::update()
{
	helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2f>& src = d_src.getValue();
	for (auto pt : src)
		dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}

template <>
void PointVectorConverter<common::cvKeypoint, defaulttype::Vec2f>::update()
{
	helper::vector<defaulttype::Vec2f>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(defaulttype::Vec2f(kp.pt.x, kp.pt.y));
}


defaulttype::Vector3 get3DFrom2DPosition(double x, double y, defaulttype::Mat3x4d P, double depth)
{
	defaulttype::Matrix3 C;
	defaulttype::Vector3 T;

	for (unsigned j = 0; j < 3; j++)
	{
		for (unsigned i = 0; i < 3; i++)
		{
			C[j][i] = P[j][i];
		}
		T[j] = P[j][3];
	}

	if (defaulttype::oneNorm(C) == 0) return defaulttype::Vector3();

	defaulttype::Matrix3 iC;
	iC.invert(C);

	defaulttype::Vector3 camera_pos = -iC * T;

	return iC * defaulttype::Vector3(x, y, 1) * depth + camera_pos;
}

template <>
void PointVectorConverter<defaulttype::Vec2f, defaulttype::Vec3f>::update()
{
	helper::vector<defaulttype::Vec3f>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2f>& src = d_src.getValue();
	for (auto pt : src) dst.push_back(get3DFrom2DPosition(pt.x(), pt.y(), d_projection.getValue(), d_depth.getValue()));
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
