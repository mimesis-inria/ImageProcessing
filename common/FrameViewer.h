#ifndef SOFA_OR_ACQUISITOR_FRAMEVIEWER_H
#define SOFA_OR_ACQUISITOR_FRAMEVIEWER_H

#include "initPlugin.h"

#include <SofaORCommon/cvMat.h>

#include <SofaORCommon/ImplicitDataEngine.h>

#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/Quater.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class FrameViewer : virtual public common::ImplicitDataEngine
{
 public:
	SOFA_CLASS(FrameViewer, common::ImplicitDataEngine);

 public:
  FrameViewer();
  virtual ~FrameViewer();

	void init();
  void update();
	void draw(const core::visual::VisualParams*);
	void computeBBox(const core::ExecParams* params, bool);

	Data<common::cvMat> d_frame;
	Data<helper::vector<defaulttype::Vector3> > d_corners;
	Data<helper::OptionsGroup> d_mode;
	Data<float> d_alpha;

 private:
	void perspectiveDraw();
	void orthoDraw();
	void bindGlTexture(const std::string& imageString);
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_ACQUISITOR_FRAMEVIEWER_H
