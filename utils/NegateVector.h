#ifndef SOFACV_UTILS_NEGATEVECTOR_H
#define SOFACV_UTILS_NEGATEVECTOR_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include <sofa/defaulttype/Mat.h>

namespace sofacv
{
namespace utils
{
class SOFA_IMAGEPROCESSING_API NegateVector : public ImplicitDataEngine
{
 public:
  SOFA_CLASS(NegateVector, ImplicitDataEngine);

  NegateVector()
      : d_src(initData(&d_src, "input_position", "input vector to negate")),
        //        d_srcRot(
        //            initData(&d_srcRot, "input_rotation", "input rotation to
        //            negate")),
        d_dst(
            initData(&d_dst, "output_position", "negated output vector")) /*,*/
  //        d_dstRot(
  //            initData(&d_dstRot, "output_rotation", "negated output vector"))
  {
  }

  virtual ~NegateVector() override {}
  void init() override;
  void Update() override;
  // INPUTS
  sofa::Data<sofa::defaulttype::Vector3> d_src;
  sofa::Data<sofa::defaulttype::Matrix3> d_srcRot;
  // OUTPUTS
  sofa::Data<sofa::defaulttype::Vector3> d_dst;
  //  sofa::Data<sofa::defaulttype::Matrix3> d_dstRot;
};

}  // namespace utils
}  // namespace sofacv

#endif  // SOFACV_UTILS_NEGATEVECTOR_H
