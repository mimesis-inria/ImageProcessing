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
        d_P(initData(&d_P, "points_out", "output projected points")),
        d_method(initData(&d_method, "method",
                          "projection method (either ORTHO or PARALLEL)"))
  {
    sofa::helper::OptionsGroup* t = d_method.beginEdit();
    t->setNames(2, "ORTHO", "PARALLEL");
    t->setSelectedItem(0);
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

  // Project the line defined by the point p1 and the direction vector d1
  // On the plane defined by the point p2 and the normal n2
  void linePlaneIntersection(const Vec3d& p1, const Vec3d& d1, const Vec3d& p2,
                             const Vec3d& n2, Vec3d& P)
  {
    double lambda =
        sofa::defaulttype::dot(p2 - p1, n2) / sofa::defaulttype::dot(d1, n2);
    P = lambda * d1 + p1;
  }

  // Finds the normal of the plane to which at least d1 and 1 point of d2 belong
  // to
  void lines2PlaneNormal(const Vec3d& d1, const Vec3d& d2, Vec3d& n)
  {
    n = d1.cross(d1.cross(d2));
  }

  void update()
  {
    Vec3d C = l_cam->getPosition();
    const vector<Vec3d>& S = d_S.getValue();
    const vector<Vec3d>& V = d_V.getValue();

    if (S.size() != V.size())
    {
      m_isMapped = false;
      return;
    }

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

    if (!m_isMapped)
    {
      m_pMap.clear();
      m_pMap.resize(V.size());
    }

    //     Clean P and resize it to the number of slaves
    vector<Vec3d> /*&*/ P /* = *d_P.beginWriteOnly()*/;
    //    P.clear();
    P.resize(V.size());

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
    m_isMapped = false;

    if (d_method.getValue().getSelectedItem() == "PARALLEL")
    {
      // S now contains the master points list, and V contains the slaves list
      // m_pMap contains the indexes of S to match V
      vector<Vec3d> Pbis;
      Pbis.resize(P.size());

      // normal of our image plane (also dP)
      Vec3d nC = l_cam->getRotationMatrix().line(2);

      for (size_t i = 0; i < V.size(); ++i)
      {
        // direction vector of line LP
        Vec3d dP = nC;
        // direction vector of the line L1
        //        Vec3d d1 = (V[i] - C).normalized();
        // direction vector of the line L2
        Vec3d d2 = (S[m_pMap[i]] - C).normalized();

        // Slave point projected on the image plane
        Vec3d P1;
        linePlaneIntersection(V[i], nC, S[m_pMap[i]], nC, P1);

        // normal of the plane to which Lp and s2 belong
        Vec3d nP;
        lines2PlaneNormal(dP, d2, nP);

        // The projection of s1 on the line L2
        Vec3d s2;
        linePlaneIntersection(C, d2, P1, nP, s2);

        Pbis[i] = s2;
      }
      d_P.setValue(Pbis);
    }
    else if (d_method.getValue().getSelectedItem() == "ORTHO")
    {
      std::cout << "PROJECTION" << std::endl;
      d_P.setValue(P);
    }
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
  sofa::Data<sofa::helper::OptionsGroup> d_method;
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
