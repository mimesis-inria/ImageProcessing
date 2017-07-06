#ifndef SOFA_OR_ACQUISITOR_FRAMEVIEWER_H
#define SOFA_OR_ACQUISITOR_FRAMEVIEWER_H

#include "initPlugin.h"

#include <SofaORCommon/cvMat.h>

#include <SofaORCommon/ImplicitDataEngine.h>

#include <SofaBaseVisual/VisualModelImpl.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/Quater.h>

#include <opencv2/opencv.hpp>

namespace sofaor
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
	void draw(const sofa::core::visual::VisualParams*);
	void computeBBox(const sofa::core::ExecParams* params, bool);

	sofa::Data<common::cvMat> d_frame;
	sofa::Data<sofa::helper::vector<sofa::defaulttype::Vector3> > d_corners;
	sofa::Data<sofa::helper::OptionsGroup> d_mode;
	sofa::Data<float> d_alpha;

 private:
	void perspectiveDraw();
	void orthoDraw();
	void bindGlTexture(const std::string& imageString);
};

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_ACQUISITOR_FRAMEVIEWER_H
