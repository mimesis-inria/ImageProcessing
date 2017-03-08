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
void PointVectorConverter<common::cvKeypoint, defaulttype::Vec2i>::update()
{
	helper::vector<defaulttype::Vec2i>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<common::cvKeypoint>& src = d_src.getValue();
	for (auto kp : src) dst.push_back(defaulttype::Vec2i(kp.pt.x, kp.pt.y));
}

template <>
void PointVectorConverter<defaulttype::Vec2i, common::cvKeypoint>::update()
{
	helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<defaulttype::Vec2i>& src = d_src.getValue();
	for (auto pt : src)
		dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
}

template <>
void PointVectorConverter<int, double>::update()
{
	helper::vector<double>& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<int>& src = d_src.getValue();
	for (auto val : src) dst.push_back(double(val));
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
