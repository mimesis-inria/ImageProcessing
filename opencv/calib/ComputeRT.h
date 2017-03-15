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
				d_P1(initData(&d_P1, "projMat1",
											"4x4 Projection matrix of the reference camera", true, false)),
				d_P2(initData(&d_P2, "projMat2",
											"4x4 Projection matrix of the second camera", true, false)),
				d_P1_out(initData(&d_P1_out, "projMat1_out",
											"3x3 Projection matrix of the reference camera", true, false)),
				d_P2_out(initData(&d_P2_out, "projMat2_out",
											"3x3 Projection matrix of the second camera", true, false)),
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
		// hard coded image resolution
		int h = 1000;
		int w = 1000;

		defaulttype::Vector3 dir1 = d_dir1.getValue() - d_pos1.getValue();
		defaulttype::Vector3 dir2 = d_dir2.getValue() - d_pos2.getValue();
		defaulttype::Matrix3 R;
		R.x() = dir1.normalized();
		R.z() = dir1.cross(dir2).normalized();
		R.y() = -(R.z().cross(dir1).normalized());

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
			const defaulttype::Matrix4 & mp1 = d_P1.getValue();

			double fx = mp1[0][0] ;
			double s = mp1[1][0] ;
			double x0 = mp1[2][0] ;
			double fy = mp1[1][1] ;
			double y0 = mp1[2][1] ;

			defaulttype::Vec2f oglCenter(0.0,0.0);
			defaulttype::Matrix3 P1;
			//see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
			P1[0][0] =  0.5 * w * fx;
			P1[0][1] = -0.5 * w * s;
			P1[0][2] = -0.5 * (w * x0 - 2.0 * oglCenter[0] - w);

			P1[1][1] =  0.5 * h * fy;
			P1[1][2] =  0.5 * (h * y0 - 2.0 * oglCenter[1] + h);

			P1[2][2] =  1.0;

			const defaulttype::Matrix4 & mp2 = d_P2.getValue();

			fx = mp2[0][0] ;
			s = mp2[1][0] ;
			x0 = mp2[2][0] ;
			fy = mp2[1][1] ;
			y0 = mp2[2][1] ;

			defaulttype::Matrix3 P2;
			//see https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL
			P2[0][0] =  0.5 * w * fx;
			P2[0][1] = -0.5 * w * s;
			P2[0][2] = -0.5 * (w * x0 - 2.0 * oglCenter[0] - w);

			P2[1][1] =  0.5 * h * fy;
			P2[1][2] =  0.5 * (h * y0 - 2.0 * oglCenter[1] + h);

			P2[2][2] =  1.0;

			d_P1_out.setValue(P1);
			d_P2_out.setValue(P2);

			// Computing Fundamental matrix:
			//  -T         -1
			//P2   * E * P1
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
	Data<defaulttype::Matrix4> d_P1;
	Data<defaulttype::Matrix4> d_P2;

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
