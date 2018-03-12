/******************************************************************************
*       SOFAOR, SOFA plugin for the Operating Room, development version       *
*                        (c) 2017 INRIA, MIMESIS Team                         *
*                                                                             *
* This program is a free software; you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 1.0 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Bruno Marques and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact-mimesis@inria.fr                               *
******************************************************************************/

#include <SceneCreator/SceneCreator.h>

#include <SofaTest/Sofa_test.h>

#include <SofaSimulationGraph/DAGSimulation.h>
#include <sofa/simulation/Simulation.h>
using sofa::core::objectmodel::BaseObject;
using sofa::simulation::Simulation;
using sofa::simulation::Node;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML;
using sofa::core::ExecParams;

#include "ProcessOR/camera/common/CameraSettings.h"
using sofaor::processor::cam::CameraSettings;
using sofa::defaulttype::Vec2i;
using sofa::defaulttype::Vector3;
using sofa::defaulttype::Matrix3;
using sofa::defaulttype::Matrix4;
using sofa::defaulttype::Mat3x4d;

namespace sofa
{
struct CameraSettings_test : public sofa::Sofa_test<>
{
  sofa::simulation::Node::SPtr root;
  CameraSettings::SPtr cam;
  Vector3 A, B, C, D, P;

  CameraSettings_test() {}

  void SetUp()
  {
    using modeling::addNew;
    simulation::Simulation* simu;
    sofa::simulation::setSimulation(
        simu = new sofa::simulation::graph::DAGSimulation());

    root = simu->createNewGraph("root");

    // CameraSettings creation:
    cam = addNew<CameraSettings>(root);
    cam->setName("cam");
    cam->setImageSize(Vec2i(1280, 720), false);
    cam->setFocalDistance(1.0);

    //    cam->setGLProjection(Matrix4(Matrix4::Line(1,0,0, 0), Matrix4::Line(0,
    //    1, 0, 0),
    //                                 Matrix4::Line(0, 0, 1, 0),
    //                                 Matrix4::Line(0, 0, 0, 1)));
    //    cam->setGLModelview(Matrix4(Matrix4::Line(1,0,0, 0), Matrix4::Line(0,
    //    1, 0, 0),
    //                                 Matrix4::Line(0, 0, 1, 0),
    //                                 Matrix4::Line(0, 0, 0, 1)));

    // The 4 pre-calculated corner points of the Image plane projected in f=1.0:
    A = Vector3(0.22360679507255554199, 1.56524756550788879395, 0.28125);
    B = Vector3(1.11803397536277770996, 1.11803397536277770996, 0.28125);
    C = Vector3(1.11803397536277770996, 1.11803397536277770996, -0.28125);
    D = Vector3(0.22360679507255554199, 1.56524756550788879395, -0.28125);

    // A point that we will use to test the projection with 2D coord: [960;180]
    P = Vector3(0.22360679507255554199, 1.56524756550788879395, 0.28125);

    //    sofa::simulation::getSimulation()->init(this->root.get());
  }

  void TearDown() {}
};

TEST_F(CameraSettings_test, KRTBuild)
{
  cam->setPosition(
      Vector3(0.22360679507255554199, 0.44721359014511108398, -0.0), false);
  cam->setRotationMatrix(
      Matrix3(Matrix3::Line(0.89443, -0.44721, 0), Matrix3::Line(0, 0, -1),
              Matrix3::Line(0.44721, 0.89443, 0)),
      false);
  cam->setIntrinsicCameraMatrix(
      Matrix3(Matrix3::Line(1280, 0, 640), Matrix3::Line(0, 1280, 360),
              Matrix3::Line(0, 0, 1)),
      true);

  Mat3x4d M = Mat3x4d(
      Matrix4::Line(1431.0848, 0.006399999999985084, 0, -320.0031477720261),
      Matrix4::Line(160.9956, 321.9948, -1280, -180.0001606528402),
      Matrix4::Line(0.44721, 0.8944299999999999, 0, -0.5000004462578893));

  Mat3x4d r = M - this->cam->getProjectionMatrix();
  double res = 0;
  for (size_t i = 0; i < 12; ++i) res += std::fabs(r.ptr()[i]);
  EXPECT_TRUE(res < 1e-10);


  cam->buildFromKRT();
  r = M - this->cam->getProjectionMatrix();
  res = 0;
  for (size_t i = 0; i < 12; ++i) res += std::fabs(r.ptr()[i]);
  EXPECT_TRUE(res < 1e-10);


  cam->setPosition(Vector3(0, 0, 0.0), false);

  cam->setRotationMatrix(Matrix3(Matrix3::Line(1, 0, 0), Matrix3::Line(0, 1, 0),
                                 Matrix3::Line(0, 0, 1)),
                         true);

  Mat3x4d M2 =
      Mat3x4d(Matrix4::Line(1280, 0, 640, 0), Matrix4::Line(0, 1280, 360, 0),
              Matrix4::Line(0, 0, 1, 0));

  r = M2 - this->cam->getProjectionMatrix();
  res = 0;
  for (size_t i = 0; i < 12; ++i) res += fabs(r.ptr()[i]);
  EXPECT_TRUE(res < 1e-10);

}

// TEST_F(CameraSettings_test, buildFromIntrinsicCamPosLookAtAndUpVector)
//{
//  this->cam->buildFromIntrinsicCamPosLookAtAndUpVector();
//}

// TEST_F(CameraSettings_test, buildFromIntrinsicCamPosUpVectorAndFwdVector)
//{
//  this->buildFromIntrinsicCamPosUpVectorAndFwdVector();
//}

// TEST_F(CameraSettings_test, buildFromCamPosAndImageCorners)
//{
//  this->buildFromCamPosAndImageCorners();
//}

// TEST_F(CameraSettings_test, buildFromM) { this->buildFromM(); }

// TEST_F(CameraSettings_test, buildFromKRT) { this->buildFromKRT(); }

// TEST_F(CameraSettings_test, buildFromOpenGL) { this->buildFromOpenGL(); }

// TEST_F(CameraSettings_test, buildFromOpenGLContext)
//{
//  this->buildFromOpenGLContext();
//}

}  // namespace sofa
