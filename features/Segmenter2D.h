#ifndef SOFACV_FEATURES_SEGMENTER2D_H
#define SOFACV_FEATURES_SEGMENTER2D_H

#include "ImageProcessingPlugin.h"
#include <SofaCV/SofaCV.h>
#include <sofa/helper/SVector.h>

namespace sofacv
{
namespace features
{
class SOFA_IMAGEPROCESSING_API Segmenter2D : public ImageFilter
{
 public:
  SOFA_CLASS(Segmenter2D, ImageFilter);

  sofa::Data<std::string> d_regionLabel;

  // INPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_points;

  // OUTPUTS
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_regionPoly;
  sofa::Data<sofa::helper::vector<sofa::defaulttype::Vec2i> > d_regionPoints;

  Segmenter2D();

  virtual void init() override;

  virtual void Update() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;

  // Mouse controls
  typedef void (Segmenter2D::*StateFunction)(int, int, int, int);
  void freeMove(int event, int x, int y,
                int flags);  // mouse is moving, buttons are not pressed
  void capture(int event, int x, int y,
               int flags);  // left is down, capturing motion
  void capturePaused(int event, int x, int y,
                     int flags);  // left is down, capturing motion
  void stopping(int event, int x, int y, int flags);  // right is down, stopping

  StateFunction m_activeState;
  void setMouseState(StateFunction f) { m_activeState = f; }
  void mouseCallback(int event, int x, int y, int flags) override;

  std::list<cv::Point2i> m_poly;
};

SOFA_DECL_CLASS(Segmenter2D)

int Segmenter2DClass =
    sofa::core::RegisterObject("Manual segmentation component")
        .add<Segmenter2D>();

}  // namespace features
}  // namespace sofacv
#endif  // SOFACV_FEATURES_SEGMENTER2D_H
