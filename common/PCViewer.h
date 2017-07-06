#ifndef SOFA_OR_ACQUISITOR_PCVIEWER_H
#define SOFA_OR_ACQUISITOR_PCVIEWER_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>

namespace sofaor
{
namespace processor
{
class PCViewer : public common::ImplicitDataEngine
{
	typedef sofa::defaulttype::Vector3 Vector3;
	typedef sofa::defaulttype::Vec<3, uint8_t> Vec3b;

 public:
	SOFA_CLASS(PCViewer, common::ImplicitDataEngine);

 public:
  PCViewer();
  virtual ~PCViewer();

  void init();
  void update();
	void computeBBox(const sofa::core::ExecParams*, bool);
	void draw(const sofa::core::visual::VisualParams*);
  void reinit();

	sofa::Data<double> d_size;
	// INPUTS
	sofa::Data<sofa::helper::vector<Vector3> > d_positions;
	sofa::Data<sofa::helper::vector<Vec3b> > d_colors;

 private:
	std::vector<sofa::defaulttype::Vec4f> m_colors;
	std::vector<Vector3> m_positions;
};

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_ACQUISITOR_PCVIEWER_H
