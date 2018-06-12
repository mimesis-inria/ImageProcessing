#include "NegateVector.h"

namespace sofacv
{
namespace utils
{

void NegateVector::init()
{
  addInput(&d_src);
//    addInput(&d_srcRot);
  addOutput(&d_dst);
}

void NegateVector::Update()
{
  d_dst.setValue(-d_src.getValue());
//    sofa::defaulttype::Matrix3 rev;
//    d_dstRot.setValue(-d_srcRot.getValue().transposed());
//    d_dst.setValue(d_dstRot.getValue() * d_dst.getValue());
}


SOFA_DECL_CLASS(NegateVector)

int NegateVectorClass =
    sofa::core::RegisterObject("component to negate a defaulttype::Vector3")
        .add<NegateVector>();

}  // namespace utils
}  // namespace sofacv
