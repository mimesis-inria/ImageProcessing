#ifndef SOFACV_CAM_PROJECTPOINTS_H
#define SOFACV_CAM_PROJECTPOINTS_H

#include "ImageProcessingPlugin.h"

#include <SofaCV/SofaCV.h>
#include "CameraSettings.h"

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
/**
 * @brief The ProjectPoints class
 *
 * Projects a 2D point cloud in 3D or vice-versa using a linked CameraSettings
 * component
 */
class SOFA_IMAGEPROCESSING_API ProjectPoints : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      ProjectPoints, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

  typedef typename sofa::defaulttype::Vector3 Vector3;
  typedef typename sofa::defaulttype::Vector2 Vector2;

 public:
  SOFA_CLASS(ProjectPoints, ImplicitDataEngine);

  ProjectPoints();

  ~ProjectPoints() {}
  void init() override;

  virtual void doUpdate() override;

  CamSettings l_cam;           ///< Linked CameraSettings
  sofa::Data<bool> d_2Dto3D;   ///< projection direction (2Dto3D or 3Dto2D)
  sofa::Data<double> d_depth;  ///< focal distance for 2Dto3D projection (-1 to
                               /// take CameraSettings focal distance)
  sofa::Data<sofa::helper::vector<Vector3> >
      d_Pts3D;  ///< [INPUT / OUTPUT] set of 3D points
  sofa::Data<sofa::helper::vector<Vector2> >
      d_Pts2D;  ///< [INPUT / OUTPUT] set of 2D points
};

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_PROJECTPOINTS_H
