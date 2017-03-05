#ifndef SOFA_OR_PROCESSOR_COMPUTERT_H
#define SOFA_OR_PROCESSOR_COMPUTERT_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofa
{
namespace OR
{
namespace processor
{
class ComputeRT : public common::ImplicitDataEngine
{
 public:
  SOFA_CLASS(ComputeRT, common::ImplicitDataEngine);

  ComputeRT()
      : d_pos1(initData(
            &d_pos1, "pos1",
            "reference position (reference camera, usually the Left one)")),
        d_pos2(
            initData(&d_pos2, "pos2",
                     "position of the second camera (usually the right one)")),
        d_dir1(initData(&d_dir1, "dir1", "position of the reference camera's target")),
        d_dir2(initData(&d_dir2, "dir2", "position of the second camera's target")),
        //        d_up1(initData(&d_up1, "up1",
        //                       "up direction vector of the reference
        //                       camera")),
        //        d_up2(initData(&d_up2, "up2",
        //                       "up direction vector of the second camera")),
        d_R(initData(&d_R, "R", "output Rotation matrix")),
        d_t(initData(&d_t, "t", "output translation vector"))
  {
      addAlias(&d_R, "R_out");
      addAlias(&d_t, "t_out");
  }

  ~ComputeRT() {}

  void init()
  {
    addInput(&d_pos1);
    addInput(&d_pos2);
    addInput(&d_dir1);
    addInput(&d_dir2);
//    addInput(&d_up1);
//    addInput(&d_up2);
    addOutput(&d_R);
    addOutput(&d_t);
  }

  void update()
  {
    defaulttype::Vector3 dir1 = d_dir1.getValue() - d_pos1.getValue();
    defaulttype::Vector3 dir2 = d_dir2.getValue() - d_pos2.getValue();
    defaulttype::Matrix3 R;
    R.x() = dir1.normalized();
    R.z() = dir1.cross(dir2).normalized();
    R.y() = R.z().cross(dir1).normalized();

    d_R.setValue(R);
    d_t.setValue(d_pos2.getValue() - d_pos1.getValue());
  }

  // INPUTS
  Data<defaulttype::Vector3> d_pos1;
  Data<defaulttype::Vector3> d_pos2;
  Data<defaulttype::Vector3> d_dir1;
  Data<defaulttype::Vector3> d_dir2;
  //  Data<defaulttype::Vector3> d_up1;
  //  Data<defaulttype::Vector3> d_up2;

  // OUTPUTS
  Data<defaulttype::Matrix3> d_R;
  Data<defaulttype::Vector3> d_t;
};

SOFA_DECL_CLASS(ComputeRT)

int ComputeRTClass = core::RegisterObject(
                         "Simple component computing the Rotation matrix and "
                         "translation vector between 2 cameras")
                         .add<ComputeRT>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_COMPUTERT_H
