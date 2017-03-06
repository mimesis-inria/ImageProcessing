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
				d_dir1(initData(&d_dir1, "dir1",
												"position of the reference camera's target")),
				d_dir2(initData(&d_dir2, "dir2",
												"position of the second camera's target")),
				d_P1(initData(&d_P1, "projMat1",
											"Projection matrix of the reference camera", true, true)),
				d_P2(initData(&d_P2, "projMat2",
											"Projection matrix of the second camera", true, true)),
				d_R(initData(&d_R, "R", "output Rotation matrix")),
				d_t(initData(&d_t, "t", "output translation vector")),
				d_E(initData(&d_E, "E", "output Essential matrix")),
				d_F(initData(&d_F, "F", "output Fundamental matrix"))
	{
		addAlias(&d_R, "R_out");
		addAlias(&d_t, "t_out");
		addAlias(&d_E, "E_out");
		addAlias(&d_F, "F_out");
	}

  ~ComputeRT() {}
  void init()
  {
		addInput(&d_P1);
		addInput(&d_P2);
		addInput(&d_pos1);
    addInput(&d_pos2);
    addInput(&d_dir1);
    addInput(&d_dir2);
    addOutput(&d_R);
    addOutput(&d_t);
		addOutput(&d_E);
		addOutput(&d_F);
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

		defaulttype::Vec3d t = d_t.getValue();
		// COMPUTING F:
		// F = PL * R*[t]x * PR
		// Where [t]x is the matrix representation of the cross product with t
		double p[9] = {0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0};
		defaulttype::Matrix3 T(p);
		d_E.setValue(R * T);

		if (d_P1.isSet() && d_P2.isSet())
		{
			// Computing Fundamental matrix:
			// F = P2' * E * P1
			d_F.setValue(d_P2.getValue().transposed() * d_E.getValue() *
									 d_P1.getValue());
		}
	}

  // INPUTS
  Data<defaulttype::Vector3> d_pos1;
  Data<defaulttype::Vector3> d_pos2;
  Data<defaulttype::Vector3> d_dir1;
  Data<defaulttype::Vector3> d_dir2;
	Data<defaulttype::Matrix3> d_P1;
	Data<defaulttype::Matrix3> d_P2;

  // OUTPUTS
  Data<defaulttype::Matrix3> d_R;
  Data<defaulttype::Vector3> d_t;
	Data<defaulttype::Matrix3> d_E;
	Data<defaulttype::Matrix3> d_F;
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
