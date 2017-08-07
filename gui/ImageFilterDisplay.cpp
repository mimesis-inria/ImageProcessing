#include <GL/glew.h>
#include "ImageFilterDisplay.h"

#include <SofaQtQuickGUI/SofaScene.h>

#include <QtGui/QOpenGLFramebufferObject>


namespace sofaor
{
namespace processor
{
FilterRenderer::FilterRenderer(const ImageFilterDisplay *imageFilterDisplay)
    : m_imageFilterDisplay(imageFilterDisplay)
{
}

void FilterRenderer::render()
{
  if (!m_imageFilterDisplay) return;
  if (!m_imageFilterDisplay->imageFilterModel()) return;

  glUseProgram(0);
  glClearColor(0, 0, 1, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  this->m_imageFilterDisplay->imageFilterModel()->display();

  update();
}

QOpenGLFramebufferObject *FilterRenderer::createFramebufferObject(
    const QSize &size)
{
  QOpenGLFramebufferObjectFormat format;
  format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
  format.setSamples(4);
  return new QOpenGLFramebufferObject(size, format);
}

/////////////
QQuickFramebufferObject::Renderer *ImageFilterDisplay::createRenderer() const
{
  return new FilterRenderer(this);
}

void ImageFilterDisplay::update() {}

}  // namespace processor
}  // namespace sofaor
