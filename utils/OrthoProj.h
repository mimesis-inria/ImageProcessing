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

#ifndef SOFA_OR_PROCESSOR_ORTHOPROJ_H
#define SOFA_OR_PROCESSOR_ORTHOPROJ_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include "ProcessOR/camera/common/CameraSettings.h"

#include <limits>

namespace sofaor
{
namespace processor
{
namespace utils
{
using sofa::helper::vector;
using sofa::defaulttype::Vec3d;

class OrthoProj : public common::ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      OrthoProj, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(OrthoProj, common::ImplicitDataEngine);

  OrthoProj()
      : l_cam(initLink("cam",
                       "link to CameraSettings component to reproject 2D "
                       "points in 3D n vice versa")),
        d_S(initData(&d_S, "observations", "Observation points")),
        d_V(initData(&d_V, "slavePts",
                     "slave points to project on the line cam -> observation")),
        d_P(initData(&d_P, "points_out", "output projected points"))
  {
  }

  ~OrthoProj() {}
  void init()
  {
    if (!l_cam.get())
      msg_error(getName() + "::init()") << "Error: No camera link set. "
                                           "Please use attribute 'cam' "
                                           "to define one";

    addInput(&d_S);
    addInput(&d_V);
    addOutput(&d_P);
    m_isMapped = false;
  }

  void update()
  {
    Vec3d C = l_cam->getPosition();
    const vector<Vec3d>& S = d_S.getValue();
    const vector<Vec3d>& V = d_V.getValue();

    if (S.size() != V.size())
    {
      msg_warning(getName() + "update()")
          << "Warning: uneven number of points to project and lines of sight";
      return;
    }

    // Clean P and resize it to the number of slaves
    vector<Vec3d>& P = *d_P.beginWriteOnly();
    P.clear();
    P.resize(V.size());

    // Compute the projection of each slave on each line of sight
    std::vector<std::vector<Vec3d> > PtsMap;
    for (size_t j = 0; j < V.size(); ++j)
    {
      std::vector<Vec3d> ptsP;
      for (size_t i = 0; i < S.size(); ++i)
      {
        Vec3d pt;
        Vec3d s = S[i];
        Vec3d v = V[j];
        // place camera in origin (0,0,0)
        if (C != Vec3d(0, 0, 0))
        {
          s = s - C;
          v = v - C;
        }
        // Orthogonal projection of V on C->S
        pt = (sofa::defaulttype::dot(v, s) / sofa::defaulttype::dot(s, s)) * s;
        // replace P in world coordinates C != (0,0,0)
        if (C != Vec3d(0, 0, 0))
        {
          pt += C;
        }
        ptsP.push_back(pt);
      }
      PtsMap.push_back(ptsP);
    }

    if (!m_isMapped) m_pMap.resize(V.size());

    for (size_t i = 0; i < V.size(); ++i)
    {
      if (m_isMapped)
      {
        // If we already mapped the pairs, then we directly set P[i] to its
        // match
        P[i] = PtsMap[i][m_pMap[i]];
      }
      else
      {
        // Otherwise, we keep only the closest P for each slave
        double dist = std::numeric_limits<double>::max();
        P[i] = PtsMap[i][0];
        for (size_t j = 0; j < PtsMap[i].size(); ++j)
        {
          Vec3d pt1 = PtsMap[i][j];
          Vec3d pt2 = V[i];
          double d = (pt2.x() - pt1.x()) * (pt2.x() - pt1.x()) +
                     (pt2.y() - pt1.y()) * (pt2.y() - pt1.y()) +
                     (pt2.z() - pt1.z()) * (pt2.z() - pt1.z());
          if (d < dist)
          {
            P[i] = pt1;
            dist = d;
            m_pMap[i] = j;
          }
        }
      }
    }
    m_isMapped = true;
    d_P.endEdit();
  }

  // INPUTS
  CamSettings l_cam;
  /// The points that define the line of sights on which to project V
  sofa::Data<sofa::helper::vector<Vec3d> > d_S;
  /// The points that we want to project on the line of sights camPos -> S
  sofa::Data<sofa::helper::vector<Vec3d> > d_V;
  // OUTPUTS
  /// The projected points P that belong to camPos -> V
  sofa::Data<sofa::helper::vector<Vec3d> > d_P;
  std::vector<int> m_pMap;
  bool m_isMapped;
};

SOFA_DECL_CLASS(OrthoProj)

int OrthoProjClass = sofa::core::RegisterObject(
                         "component to project the 'slaves' points on the "
                         "lines of sight 'cam' -> 'observations'")
                         .add<OrthoProj>();

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_ORTHOPROJ_H
