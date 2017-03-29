#ifndef SOFA_OR_PROCESSOR_COMPUTERT_H
#define SOFA_OR_PROCESSOR_COMPUTERT_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

#include <opencv2/opencv.hpp>

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
				d_up1(initData(&d_up1, "up1",
												"up vector of the reference camera")),
				d_up2(initData(&d_up2, "up2",
												"up vector of the second camera")),
				d_R1(initData(&d_R1, "R1",
												"up vector of the reference camera")),
				d_R2(initData(&d_R2, "R2",
												"up vector of the second camera")),
				d_P1(initData(&d_P1, "K1", "Projection matrix of the reference camera",
											true, false)),
				d_P2(initData(&d_P2, "K2", "Projection matrix of the second camera",
											true, false)),
				d_P1_out(initData(&d_P1_out, "K1_out",
													"3x3 Projection matrix of the reference camera", true,
													false)),
				d_P2_out(initData(&d_P2_out, "K2_out",
													"3x3 Projection matrix of the second camera", true,
													false)),
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
		addInput(&d_R1);
		addInput(&d_R2);
		addInput(&d_pos1);
    addInput(&d_pos2);
		addInput(&d_dir1);
		addInput(&d_dir2);
		addInput(&d_up1);
		addInput(&d_up2);
		addOutput(&d_R);
    addOutput(&d_t);
		addOutput(&d_E);
		addOutput(&d_F);
	}

	// retieves the rotation matrix R that rotates an object who's up vector and
	// normal vector are respectively up1 and normal1, to an object who's up
	// vector and normal vector are respectivelt up2 and normal2
	void computeRotation(const defaulttype::Vector3& x0,
											 const defaulttype::Vector3& y0,
											 const defaulttype::Vector3& x1,
											 const defaulttype::Vector3& y1, defaulttype::Matrix3& R)
	{
		defaulttype::Vector3 up = x0.normalized();  // 1st camera's up vector
		defaulttype::Vector3 norm = y0.normalized();
		defaulttype::Vector3 Z(up);
		defaulttype::Vector3 Y(norm);
		defaulttype::Vector3 X(Y.cross(Z));

		up = x1.normalized();
		norm = y1.normalized();

		defaulttype::Vector3 Z2(up);
		defaulttype::Vector3 Y2(norm);
		defaulttype::Vector3 X2(Y.cross(Z));

		double r[9] = {X2 * X, Y2 * X, Z2 * X, X2 * Y, Y2 * Y,
									 Z2 * Y, X2 * Z, Y2 * Z, Z2 * Z};
		R = r;
	}

	void update()
	{

		defaulttype::Matrix3 R2inv;
		defaulttype::invertMatrix(R2inv, d_R2.getValue());

		d_R.setValue(R2inv * d_R1.getValue());

//		if (!d_R.isSet() || !d_t.isSet())
//		{
//			defaulttype::Vector3 x0 = d_up1.getValue();
			defaulttype::Vector3 A = d_pos1.getValue();
//			defaulttype::Vector3 x1 = d_up2.getValue();
			defaulttype::Vector3 B = d_pos2.getValue();
//			defaulttype::Vector3 y0 = d_dir1.getValue() - d_pos1.getValue();
//			defaulttype::Vector3 y1 = d_dir2.getValue() - d_pos2.getValue();
//			defaulttype::Matrix3 R;

//			computeRotation(x0, y0, x1, y1, R);
//			d_R.setValue(R);
			d_t.setValue(B - A);
//		}
		defaulttype::Matrix3 R = d_R.getValue();
		defaulttype::Vec3d t = d_t.getValue();
		// COMPUTING F:
		// F = PL * R*[t]x * PR
		// Where [t]x is the matrix representation of the cross product with t
		double p[9] = {0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0};
		defaulttype::Matrix3 T(p);
		d_E.setValue(R * T);

		if (d_P1.isSet() && d_P2.isSet())
		{
			const defaulttype::Matrix3& P1 = d_P1.getValue();
			const defaulttype::Matrix3& P2 = d_P2.getValue();

			d_P1_out.setValue(P1);
			d_P2_out.setValue(P2);

			// Computing Fundamental matrix:
			//  -T         -1
			// P2   * E * P1
			//

			defaulttype::Matrix3 P2_T_Inv;
			defaulttype::Matrix3 P1_Inv;

			defaulttype::invertMatrix(P2_T_Inv, P2.transposed());
			defaulttype::invertMatrix(P1_Inv, P1);
			d_F.setValue(P2_T_Inv * d_E.getValue() * P1_Inv);
		}
	}

  // INPUTS
  Data<defaulttype::Vector3> d_pos1;
  Data<defaulttype::Vector3> d_pos2;
  Data<defaulttype::Vector3> d_dir1;
  Data<defaulttype::Vector3> d_dir2;
	Data<defaulttype::Vector3> d_up1;
	Data<defaulttype::Vector3> d_up2;
	Data<defaulttype::Matrix3> d_P1;
	Data<defaulttype::Matrix3> d_P2;
	Data<defaulttype::Matrix3> d_R1;
	Data<defaulttype::Matrix3> d_R2;

  // OUTPUTS
	Data<defaulttype::Matrix3> d_P1_out;
	Data<defaulttype::Matrix3> d_P2_out;
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
