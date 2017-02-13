#ifndef SOFA_OR_PROCESSOR_SEGMENTER2D_H
#define SOFA_OR_PROCESSOR_SEGMENTER2D_H

#include "core/ImageFilter.h"

#include <sofa/helper/SVector.h>

#include <opencv2/imgproc.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class Segmenter2D : public ImageFilter
{
 public:
  SOFA_CLASS(Segmenter2D, ImageFilter);

  // INPUTS
  Data<helper::SVector<helper::SVector<common::cvDMatch> > > d_matches;
  Data<helper::vector<common::cvKeypoint> > d_keypoints;

  // OUTPUTS
  Data<helper::SVector<helper::SVector<defaulttype::Vec2i> > > d_regionPoly;
  Data<helper::SVector<helper::SVector<common::cvDMatch> > > d_regionPoints;
  Data<helper::vector<std::string> > d_regionLabel;

  Segmenter2D()
      : d_keypoints(
            initData(&d_keypoints, "keypoints", "input vector keypoints")),
        d_matches(
            initData(&d_matches, "matches",
                     "input vector of matches, referencing the keypoints")),
        d_regionPoly(
            initData(&d_regionPoly, "poly",
                     "output vector of polygons matching their label")),
        d_regionPoints(initData(&d_regionPoints, "matches_out",
                                "output vector of vectors of matches (one for "
                                "each polygon created)")),
        d_regionLabel(
            initData(&d_regionLabel, "labels",
                     "output vector of labels matching their polygons in poly"))
  {
    addAlias(&d_regionLabel, "labels_out");
    addAlias(&d_regionPoly, "poly_out");
  }

  void init()
  {
    bindInputData(&d_keypoints);
    bindInputData(&d_matches);
    addOutput(&d_regionPoly);
    addOutput(&d_regionPoints);
    addOutput(&d_regionLabel);
    ImageFilter::activateMouseCallback();
    setMouseState(&Segmenter2D::freeMove);
    ImageFilter::init();
  }

  void updateData()
  {
    update();

    if (d_regionPoly.getValue().empty()) return;

    std::vector<std::vector<cv::Point2i> > pts;
    for (const helper::vector<defaulttype::Vec2i>& poly :
         d_regionPoly.getValue())
    {
      std::vector<cv::Point2i> polygon;
      for (const defaulttype::Vec2i& pt : poly)
        polygon.push_back(cv::Point2i(pt.x(), pt.y()));
      pts.push_back(polygon);
    }
    helper::SVector<helper::SVector<common::cvDMatch> >& pointsInPoly =
        *d_regionPoints.beginWriteOnly();
    pointsInPoly.clear();

    const helper::vector<common::cvKeypoint>& kpts = d_keypoints.getValue();
    for (auto polygon : pts)
    {
      helper::SVector<common::cvDMatch> matches;
      for (auto match : d_matches.getValue())
      {
        cv::Point2f pt = kpts[match[0].queryIdx].pt;
        if (pointPolygonTest(polygon, pt, false) > 0)
          matches.push_back(match[0]);
      }
      pointsInPoly.push_back(matches);
    }
    d_regionPoints.endEdit();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
    in.copyTo(out);
    std::vector<std::vector<cv::Point2i> > pts;
    if (m_activeState == &Segmenter2D::freeMove)
    {
      if (d_regionPoly.getValue().empty()) return;
      for (const helper::vector<defaulttype::Vec2i>& poly :
           d_regionPoly.getValue())
      {
        std::vector<cv::Point2i> polygon;
        for (const defaulttype::Vec2i& pt : poly)
          polygon.push_back(cv::Point2i(pt.x(), pt.y()));
        pts.push_back(polygon);
      }
    }
    else
    {
      if (m_poly.size() < 2) return;
      pts.push_back(std::vector<cv::Point2i>(m_poly.begin(), m_poly.end()));
    }
    cv::Scalar color(255, 255, 255, 50);
    cv::fillPoly(out, pts, color);
    cv::addWeighted(in, 0.8, out, 0.2, 0.0, out);
  }

  void reinit()
  {
    updateData();
    ImageFilter::reinit();
  }

  virtual void handleEvent(sofa::core::objectmodel::Event* event)
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(event))
      this->updateData();
  }

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
  void mouseCallback(int event, int x, int y, int flags);

  std::list<cv::Point2i> m_poly;
};

SOFA_DECL_CLASS(Segmenter2D)

int Segmenter2DClass =
    core::RegisterObject("Manual segmentation component").add<Segmenter2D>();

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_SEGMENTER2D_H
