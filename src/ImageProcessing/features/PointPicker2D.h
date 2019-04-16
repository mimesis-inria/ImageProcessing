#ifndef SOFACV_FEATURES_POINTPICKER2D_H
#define SOFACV_FEATURES_POINTPICKER2D_H

#include <SofaCV/SofaCV.h>

#include "camera/common/StereoSettings.h"

#include <sofa/helper/SVector.h>

namespace sofacv
{
namespace features
{
class PointPicker2D : public ImageFilter
{
  typedef sofa::core::objectmodel::SingleLink<
      PointPicker2D, cam::StereoSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      Settings;

 public:
  SOFA_CLASS(PointPicker2D, ImageFilter);

  // INPUTS
  Settings l_cam;
  sofa::Data<int> d_whichImage;
  sofa::Data<std::string> d_getEpilinesFrom;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_points_in;
  // OUTPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_points_out;
  sofa::helper::vector<sofa::defaulttype::Vec3f> epilines;

  PointPicker2D();

  void init() override;

  virtual void doUpdate() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;

  void computeEpipolarLines();

 protected:
  PointPicker2D* m_picker;

  // Mouse controls
  typedef void (PointPicker2D::*StateFunction)(int, int, int, int);
  void freeMove(int event, int x, int y,
                int flags);  // mouse is moving, buttons are not pressed
  void capture(int event, int x, int y,
               int flags);  // left is down, capturing motion

  StateFunction m_activeState;
  void setMouseState(StateFunction f) { m_activeState = f; }
  void mouseCallback(int event, int x, int y, int flags);

 private:
  std::list<cv::Point2f> m_pointList;
};

SOFA_DECL_CLASS(PointPicker2D)

int PointPicker2DClass =
    sofa::core::RegisterObject("Manual 2D image point picker component")
        .add<PointPicker2D>();

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_POINTPICKER2D_H
