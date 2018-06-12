#ifndef SOFACV_UTILS_ORTHOPROJ_H
#define SOFACV_UTILS_ORTHOPROJ_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include "camera/common/CameraSettings.h"

#include <limits>

namespace sofacv
{
namespace utils
{
using sofa::helper::vector;
using sofa::defaulttype::Vec3d;

class SOFA_IMAGEPROCESSING_API OrthoProj : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      OrthoProj, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(OrthoProj, ImplicitDataEngine);

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

  ~OrthoProj() override {}
  void init() override;

  // Project the line defined by the point p1 and the direction vector d1
  // On the plane defined by the point p2 and the normal n2
  void linePlaneIntersection(const Vec3d& p1, const Vec3d& d1, const Vec3d& p2,
                             const Vec3d& n2, Vec3d& P);
  // Finds the normal of the plane to which at least d1 and 1 point of d2 belong
  // to
  void lines2PlaneNormal(const Vec3d& d1, const Vec3d& d2, Vec3d& n);
  void Update() override;

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

}  // namespace utils
}  // namespace sofacv

#endif  // SOFACV_UTILS_ORTHOPROJ_H
