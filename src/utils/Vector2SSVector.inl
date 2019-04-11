#include "Vector2SSVector.h"

namespace sofacv
{
namespace utils
{
template <class T>
std::string Vector2SSVector<T>::templateName(
    const Vector2SSVector<T>* /*converter*/)
{
  return std::string(sofa::defaulttype::DataTypeName<T>::name());
}

template <class T>
void Vector2SSVector<T>::doUpdate()
{
  sofa::helper::SVector<sofa::helper::SVector<T> >& dst =
      *(d_dst.beginWriteOnly());
  dst.clear();
  const sofa::helper::vector<T>& src = d_src.getValue();

  sofa::helper::SVector<T> v;
  for (auto pt : src) v.push_back(pt);
  dst.push_back(v);
}

}  // namespace utils
}  // namespace sofacv
