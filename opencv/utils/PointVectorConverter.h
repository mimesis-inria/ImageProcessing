#ifndef SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
#define SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofa
{
namespace OR
{
namespace processor
{
template <class TSrcType, class TDstType>
struct VectorTypes
{
  typedef TSrcType SrcType;
  typedef TDstType DstType;
  static const char* Name() { return "dafuk"; }
};

typedef VectorTypes<common::cvKeypoint, defaulttype::Vec2i> cvKeypoint2Sofa;
template <>
inline const char* cvKeypoint2Sofa::Name()
{
  return "cvKeypoint2Sofa";
}

typedef VectorTypes<defaulttype::Vec2i, common::cvKeypoint> Sofa2cvKeypoint;
template <>
inline const char* Sofa2cvKeypoint::Name()
{
  return "Sofa2cvKeypoint";
}


template <class VectorTypes>
class PointVectorConverter : public common::ImplicitDataEngine
{
 public:
  SOFA_CLASS(SOFA_TEMPLATE(PointVectorConverter, VectorTypes),
             common::ImplicitDataEngine);

  PointVectorConverter()
      : d_src(initData(&d_src, "points", "input vector to convert")),
        d_dst(initData(&d_dst, "points_out", "converted output vector"))
  {
  }

  ~PointVectorConverter() {}
  void init()
  {
    addInput(&d_src);
    addOutput(&d_dst);
  }

  void update();

  // INPUTS
  Data<helper::vector<typename VectorTypes::SrcType> > d_src;
  // OUTPUTS
  Data<helper::vector<typename VectorTypes::DstType> > d_dst;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
