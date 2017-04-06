#ifndef SOFA_OR_PROCESSOR_COMPUTERT_H
#define SOFA_OR_PROCESSOR_COMPUTERT_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMatUtils.h>

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
				d_up1(initData(&d_up1, "up1", "up vector of the reference camera")),
				d_up2(initData(&d_up2, "up2", "up vector of the second camera")),
				d_R1(initData(&d_R1, "R1", "up vector of the reference camera")),
				d_R2(initData(&d_R2, "R2", "up vector of the second camera")),
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
				d_F(initData(&d_F, "F", "output Fundamental matrix")),
				d_pts2d1(initData(&d_pts2d1, "pts2d1", "")),
				d_pts2d2(initData(&d_pts2d2, "pts2d2", "")),
				d_pts3d(initData(&d_pts3d, "pts3d", "OpenGL's 4x4 Projection matrix")),
				d_remapdist(initData(&d_remapdist, "remapDist", "Remap distance")),
				d_remapInd(initData(&d_remapInd, "remapInd", "Remap indices"))
	{
		addAlias(&d_R, "R_out");
		addAlias(&d_t, "t_out");
		addAlias(&d_E, "E_out");
		addAlias(&d_F, "F_out");
	}

  ~ComputeRT() {}
  void init()
  {
		addInput(&d_pts2d1);
		addInput(&d_pts2d2);
		addInput(&d_pts3d);

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

	defaulttype::Vector2 get2DFrom3DPosition(const defaulttype::Vector3& p)
	{
		const defaulttype::Mat3x4d& P =
				defaulttype::Mat3x4d(defaulttype::Vec<4, float>(1.0, 0.0, 0.0, 0.0),
														 defaulttype::Vec<4, float>(0.0, 1.0, 0.0, 0.0),
														 defaulttype::Vec<4, float>(0.0, 0.0, 1.0, 0.0));

		double rx = P[0][0] * p[0] + P[0][1] * p[1] + P[0][2] * p[2] + P[0][3];
		double ry = P[1][0] * p[0] + P[1][1] * p[1] + P[1][2] * p[2] + P[1][3];
		double rz = P[2][0] * p[0] + P[2][1] * p[1] + P[2][2] * p[2] + P[2][3];

		return defaulttype::Vector2(rx, ry) * 1.0 / rz;
	}

	void remapVectors(const helper::vector<defaulttype::Vector3>& inpos3d,
										const helper::vector<defaulttype::Vector2>& inpos2d,
										std::vector<cv::Point3f>& outpos3d,
										std::vector<cv::Point2f>& outpos2d)
	{
		if (inpos3d.empty() || inpos2d.empty()) return;

		if (d_remapdist.getValue() != 0)
		{
			helper::vector<defaulttype::Vector2> proj2d;

			for (unsigned i = 0; i < inpos3d.size(); i++)
			{
				proj2d.push_back(get2DFrom3DPosition(inpos3d[i]));
			}

			for (unsigned i = 0; i < inpos2d.size(); i++)
			{
				defaulttype::Vector2 p = inpos2d[i];

				int min = 0;
				for (unsigned j = 1; j < proj2d.size(); j++)
				{
					if ((p - proj2d[j]).norm() < (p - proj2d[min]).norm()) min = j;
				}

				if ((p - proj2d[min]).norm() < d_remapdist.getValue())
				{
					outpos3d.push_back(
							cv::Point3f(inpos3d[min][0], inpos3d[min][1], inpos3d[min][2]));
					outpos2d.push_back(cv::Point2f(inpos2d[i][0], inpos2d[i][1]));
				}
			}
		}
		else if (d_remapInd.getValue().size())
		{
			for (unsigned i = 0; i < d_remapInd.getValue().size(); i++)
			{
				const defaulttype::Vec2i& map = d_remapInd.getValue()[i];
				if (map[0] >= (int)inpos3d.size()) continue;
				if (map[1] >= (int)inpos2d.size()) continue;

				int id3d = map[0];
				int id2d = map[1];

				outpos3d.push_back(
						cv::Point3f(inpos3d[id3d][0], inpos3d[id3d][1], inpos3d[id3d][2]));
				outpos2d.push_back(cv::Point2f(inpos2d[id2d][0], inpos2d[id2d][1]));
			}
		}
		else
		{
			unsigned nbpts = std::min(inpos3d.size(), inpos2d.size());

			for (unsigned i = 0; i < nbpts; i++)
			{
				cv::Point3f pos3d(inpos3d[i][0], inpos3d[i][1], inpos3d[i][2]);
				outpos3d.push_back(pos3d);
			}

			for (unsigned i = 0; i < nbpts; i++)
			{
				cv::Point2f pos2d(inpos2d[i][0], inpos2d[i][1]);
				outpos2d.push_back(pos2d);
			}
		}
	}

	void update()
	{
		std::vector<cv::Point3f> pos3d;
		std::vector<cv::Point2f> pos2d1;
		std::vector<cv::Point2f> pos2d2;
		remapVectors(d_pts3d.getValue(), d_pts2d1.getValue(), pos3d, pos2d1);
		remapVectors(d_pts3d.getValue(), d_pts2d2.getValue(), pos3d, pos2d2);

		unsigned nbpts = pos3d.size();
		if (nbpts == 0) return;

		std::vector<std::vector<cv::Point2f> > imagePoints1;
		imagePoints1.push_back(pos2d1);
		std::vector<std::vector<cv::Point2f> > imagePoints2;
		imagePoints2.push_back(pos2d2);

		std::vector<std::vector<cv::Point3f> > objectPoints;
		objectPoints.push_back(pos3d);

		cv::Mat distCoeffs1;
		cv::Mat distCoeffs2;
		cv::Mat Rmat;
		cv::Mat Tvec;
		cv::Mat E;
		cv::Mat F;
		objectPoints[0].resize(objectPoints[0].size() / 2);
		std::cout << imagePoints1[0].size() << " == " << imagePoints2[0].size() << " == " << objectPoints[0].size() << std::endl;

		cv::Mat_<double> cam1, cam2;
		common::matrix::sofaMat2cvMat(d_P1.getValue(), cam1);
		common::matrix::sofaMat2cvMat(d_P2.getValue(), cam2);
		cv::stereoCalibrate(
				objectPoints, imagePoints1, imagePoints2, cam1, distCoeffs1, cam2,
				distCoeffs2, cv::Size(1000, 1000), Rmat, Tvec, E, F,
				cv::CALIB_FIX_INTRINSIC | cv::CALIB_USE_INTRINSIC_GUESS);

		d_R.setValue(defaulttype::Matrix3((double*)Rmat.ptr()));
		d_t.setValue(defaulttype::Vector3((double*)Tvec.ptr()));
		d_E.setValue(defaulttype::Matrix3((double*)E.ptr()));
		d_F.setValue(defaulttype::Matrix3((double*)F.ptr()));

		d_P1_out.setValue(d_P1.getValue());
		d_P2_out.setValue(d_P2.getValue());
		//		defaulttype::Matrix3 R2inv;
		//		defaulttype::invertMatrix(R2inv, d_R2.getValue());

		//		d_R.setValue(R2inv * d_R1.getValue());

		//		//		if (!d_R.isSet() || !d_t.isSet())
		//		//		{
		//		//			defaulttype::Vector3 x0 =
		// d_up1.getValue();
		//		defaulttype::Vector3 A = d_pos1.getValue();
		//		//			defaulttype::Vector3 x1 =
		// d_up2.getValue();
		//		defaulttype::Vector3 B = d_pos2.getValue();
		//		//			defaulttype::Vector3 y0 =
		// d_dir1.getValue()
		//-
		//		//d_pos1.getValue();
		//		//			defaulttype::Vector3 y1 =
		// d_dir2.getValue()
		//-
		//		//d_pos2.getValue();
		//		//			defaulttype::Matrix3 R;

		//		//			computeRotation(x0, y0, x1, y1, R);
		//		//			d_R.setValue(R);
		//		d_t.setValue(B - A);
		//		//		}
		//		defaulttype::Matrix3 R = d_R.getValue();
		//		defaulttype::Vec3d t = d_t.getValue();
		//		// COMPUTING F:
		//		// F = PL * R*[t]x * PR
		//		// Where [t]x is the matrix representation of the cross product
		// with
		// t
		//		double p[9] = {0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(),
		// t.x(),
		// 0};
		//		defaulttype::Matrix3 T(p);
		//		d_E.setValue(R * T);

		//		if (d_P1.isSet() && d_P2.isSet())
		//		{
		//			const defaulttype::Matrix3& P1 = d_P1.getValue();
		//			const defaulttype::Matrix3& P2 = d_P2.getValue();

		//			d_P1_out.setValue(P1);
		//			d_P2_out.setValue(P2);

		//			// Computing Fundamental matrix:
		//			//  -T         -1
		//			// P2   * E * P1
		//			//

		//			defaulttype::Matrix3 P2_T_Inv;
		//			defaulttype::Matrix3 P1_Inv;

		//			defaulttype::invertMatrix(P2_T_Inv, P2.transposed());
		//			defaulttype::invertMatrix(P1_Inv, P1);
		//			d_F.setValue(P2_T_Inv * d_E.getValue() * P1_Inv);
		//		}
	}

  // INPUTS
	Data<defaulttype::Matrix3> d_P1;
	Data<defaulttype::Matrix3> d_P2;

	Data<helper::vector<defaulttype::Vector3> > d_pts3d;

	Data<helper::vector<defaulttype::Vector2> > d_pts2d1;
	Data<helper::vector<defaulttype::Vector2> > d_pts2d2;
	Data<double> d_remapdist;
	Data<helper::vector<defaulttype::Vec2i> > d_remapInd;

  Data<defaulttype::Vector3> d_pos1;
  Data<defaulttype::Vector3> d_pos2;
  Data<defaulttype::Vector3> d_dir1;
  Data<defaulttype::Vector3> d_dir2;
	Data<defaulttype::Vector3> d_up1;
	Data<defaulttype::Vector3> d_up2;
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
