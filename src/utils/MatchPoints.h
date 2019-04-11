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

#ifndef SOFA_OR_PROCESSOR_MATCHPOINTLINE_H
#define SOFA_OR_PROCESSOR_MATCHPOINTLINE_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMatUtils.h>
#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualParams.h>
#include "ProcessOR/camera/common/CameraSettings.h"

#include <limits>

namespace sofaor
{
namespace processor
{
namespace utils
{
using sofa::defaulttype::Vec2d;
using sofa::defaulttype::Vec3d;
using sofa::helper::vector;
using namespace sofaor::common::matrix;

class MatchPoints : public common::ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      MatchPoints, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(MatchPoints, common::ImplicitDataEngine);

  MatchPoints()
      : l_cam(initLink("cam",
                       "link to CameraSettings component to reproject 2D "
                       "points in 3D n vice versa")),
        d_master(initData(&d_master, "masterPoints", "Observation points")),
        d_slave(
            initData(&d_slave, "slavePoints",
                     "slave points to project on the line cam -> observation")),
        d_masterMatches(
            initData(&d_masterMatches, "indices",
                     "inlier master point indices ordered by slave match")),
        d_threshold(initData(&d_threshold, 30.0, "threshold",
                             "minimum distance to to filter outliers"))
  {
    //    sofa::helper::OptionsGroup* t = d_method.beginEdit();
    //    t->setNames(2, "ORTHO", "PARALLEL");
    //    t->setSelectedItem(0);
  }

  ~MatchPoints() {}
  void init()
  {
    if (!l_cam.get())
      msg_error(getName() + "::init()") << "Error: No camera link set. "
                                           "Please use attribute 'cam' "
                                           "to define one";

    addInput(&d_master);
    addInput(&d_slave);
    addInput(&d_threshold);
    addOutput(&d_masterMatches);
    update();
  }

  void update()
  {
    const vector<Vec2d>& master = d_master.getValue();
    const vector<Vec3d>& slave3d = d_slave.getValue();

    // Project slaves in 2D (matching is 2D / 2D)
    vector<Vec2d> slave;
    for (const auto& pt : slave3d)
      slave.push_back(l_cam->get2DFrom3DPosition(pt));

    // Compute the square distance of each slave to each master
    vector<vector<double> > distMap;
    for (size_t j = 0; j < slave.size(); ++j)
    {
      vector<double> dist2vec;
      for (size_t i = 0; i < master.size(); ++i)
      {
        Vec2d m = master[i];
        Vec2d s = slave[j];

        dist2vec.push_back((m[0] - s[0]) * (m[0] - s[0]) + (m[1] - s[1]) * (m[1] - s[1]));
      }
      distMap.push_back(dist2vec);
    }

    // Match by min sqdistance (naive "first in first served"
    // approach...)
    double threshold =
        d_threshold.getValue();  // maximum distance in px between 2 matches
    vector<int>& matchIndices = *d_masterMatches.beginEdit();
    matchIndices.clear();
    for (size_t i = 0; i < slave.size(); ++i)
    {
      double minDist = threshold;
      for (size_t j = 0; j < master.size(); ++j)
      {
        if (distMap[i][j] < minDist)
        {
          bool taken = false;
          for (size_t k = 0; k < matchIndices.size(); ++k)
            if (k == j)
            {
              taken = true;
              break;
            }
          if (taken) continue;
          minDist = distMap[i][j];
          matchIndices.push_back(j);
        }
      }
      if (minDist == threshold)
      {
        minDist = 0;
        matchIndices.push_back(-1);
      }
    }
    d_masterMatches.endEdit();
    //    vector<Vec2d> masterMatches;
    //    for (int i = 0; i < matchIndices.size(); ++i)
    //    {
    //      if (matchIndices[i] == -1)
    //        masterMatches.push_back(slave[i]);
    //      else
    //        masterMatches[i].push_back(master[i]);
    //    }

    //    d_masterMatches.setValue(masterMatches);
  }

  // INPUTS
  CamSettings l_cam;
  /// The points that define the line of sights on which to project V
  sofa::Data<sofa::helper::vector<Vec2d> > d_master;
  /// The points that we want to project on the line of sights camPos -> S
  sofa::Data<sofa::helper::vector<Vec3d> > d_slave;
  // OUTPUTS
  /// The projected points P that belong to camPos -> V
  sofa::Data<sofa::helper::vector<int> > d_masterMatches;
  //  sofa::Data<sofa::helper::OptionsGroup> d_method;
  sofa::Data<double> d_threshold;
  sofa::Data<bool> d_outputLines;
};

SOFA_DECL_CLASS(MatchPoints)

int MatchPointsClass =
    sofa::core::RegisterObject(
        "component to match a set of 2D points with a set of 3D points")
        .add<MatchPoints>();

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_MATCHPOINTLINE_H
