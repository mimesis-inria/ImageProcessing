#include "PCViewer.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(PCViewer)

int PCViewerClass =
    core::RegisterObject("component to visualize pointclouds in Sofa")
        .add<PCViewer>();

PCViewer::PCViewer()
		: d_size(initData(&d_size, 1.0, "size", "point size")),
			d_positions(initData(&d_positions, "points", "point cloud's positions")),
      d_colors(initData(&d_colors, "colors", "point cloud's point colors"))
{
  f_listening.setValue(true);
}

PCViewer::~PCViewer() {}
void PCViewer::init()
{
  addInput(&d_positions);
	addInput(&d_colors);
	update();
}

void PCViewer::update()
{
	m_positions = d_positions.getValue();
	const helper::vector<Vec3b>& c = d_colors.getValue();
  m_colors.resize(c.size());
  for (size_t i = 0; i < c.size(); ++i)
    m_colors[i].set(c[i][2] / 255.0f, c[i][1] / 255.0f, c[i][0] / 255.0f, 1.0f);
}

void PCViewer::computeBBox(const core::ExecParams* params, bool)
{
  static const double max_double = std::numeric_limits<double>::max();

  if (m_positions.size() == 0)
  {
    double maxBBox[3] = {1, 1, 1};
    double minBBox[3] = {-1, -1, -1};
    this->f_bbox.setValue(
        params, sofa::defaulttype::TBoundingBox<double>(minBBox, maxBBox));
    return;
  }
  double maxBBox[3] = {-max_double, -max_double, -max_double};
  double minBBox[3] = {max_double, max_double, max_double};
  double r = d_size.getValue();
	for (const Vector3& p : m_positions)
    for (int i = 0; i < 3; ++i)
    {
      if (p[i] + r > maxBBox[i]) maxBBox[i] = p[i] + r;
      if (p[i] - r < minBBox[i]) minBBox[i] = p[i] - r;
    }
  this->f_bbox.setValue(
      params, sofa::defaulttype::TBoundingBox<double>(minBBox, maxBBox));
}

void PCViewer::draw(const core::visual::VisualParams*)
{
	glPointSize(d_size.getValue());
	glDisable(GL_LIGHTING);
	glBegin(GL_POINTS);

	defaulttype::Vector3 t(-1.2115, -1.2353, 3.5539);

	for (size_t i = 0; i < m_positions.size(); ++i)
	{
		Vector3 pt = m_positions[i];
		glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
		glVertex3f(pt.x(),
							 pt.y(),
							 pt.z());
	}
	glEnd();
	glPointSize(1);

	//  core::visual::DrawToolGL dt;
	//  if (m_positions.size() != m_colors.size())
	//    dt.drawPoints(m_positions, d_size.getValue(),
	//                  defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0f));
	//  else
	//    dt.drawPoints(m_positions, d_size.getValue(), m_colors);
}

void PCViewer::reinit() { update(); }
}  // namespace processor
}  // namespace OR
}  // namespace sofa
