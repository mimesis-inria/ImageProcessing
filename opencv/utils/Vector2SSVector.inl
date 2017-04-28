#include "Vector2SSVector.h"

namespace sofa
{
namespace OR
{
namespace processor
{
template <class T>
std::string Vector2SSVector<T>::templateName(
		const Vector2SSVector<T>* /*converter*/)
{
	return std::string(defaulttype::DataTypeName<T>::name());
}

template <class T>
void Vector2SSVector<T>::update()
{
	helper::SVector<helper::SVector<T> >& dst = *(d_dst.beginWriteOnly());
	dst.clear();
	const helper::vector<T>& src = d_src.getValue();

	helper::SVector<T> v;
	for (auto pt : src) v.push_back(pt);
	dst.push_back(v);
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
