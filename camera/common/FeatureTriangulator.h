#ifndef SOFACV_CAM_FEATURETRIANGULATOR_H
#define SOFACV_CAM_FEATURETRIANGULATOR_H

#include "StereoSettings.h"

#include <SofaCV/SofaCV.h>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/SVector.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
/**
 * @brief The FeatureTriangulator class
 *
 * Triangulates a set of two 2D point matches to get their 3D position.
 */
class SOFA_IMAGEPROCESSING_API FeatureTriangulator : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      FeatureTriangulator, StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      StereoCamSettings;
  typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;
  typedef sofa::defaulttype::Vec3d Vec3d;

 public:
  SOFA_CLASS(FeatureTriangulator, ImplicitDataEngine);

 public:
  FeatureTriangulator();
  virtual ~FeatureTriangulator();

  void init();
  virtual void Update() override;

  // INPUTS
  StereoCamSettings l_cam;  ///< StereoSettings component holding the two camera
                            /// settings and the fundamental matrix
  sofa::Data<sofa::helper::vector<cvKeypoint> >
      d_keypointsL;  ///< [INPUT] reference camera's keypoints
  sofa::Data<sofa::helper::vector<cvKeypoint> >
      d_keypointsR;  ///< [INPUT] second camera's keypoints
  sofa::Data<sofa::helper::vector<cvDMatch> >
      d_matches;  ///< [INPUT] matches between the keypoints, if not ordered

  sofa::Data<sofa::helper::vector<Vec3d> >
      d_pointCloud;  ///< [OUTPUT] triangulated 3D point cloud

 private:
  cv::Mat_<double> R;
  cv::Mat_<double> T;
  cv::Mat_<double> cmL;
  cv::Mat_<double> cmR;
  cv::Mat_<double> dvL;
  cv::Mat_<double> dvR;
  cv::Matx34d PL;
  cv::Matx34d PR;
};

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_FEATURETRIANGULATOR_H
