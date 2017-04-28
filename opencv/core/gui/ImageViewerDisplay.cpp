#include "ImageViewerDisplay.h"

#include <SofaQtQuickGUI/SofaScene.h>
#include <QtGui/QOpenGLFramebufferObject>

namespace sofa
{
namespace OR
{
namespace processor
{
ImageRenderer::ImageRenderer(
		const sofa::OR::processor::ImageViewerDisplay *imageViewerDisplay)
		: m_imageViewerDisplay(imageViewerDisplay)
{
}

void ImageRenderer::render()
{
	if (!m_imageViewerDisplay) return;
	if (!m_imageViewerDisplay->imageViewerModel()) return;

	glClearColor(0, 0, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	this->m_imageViewerDisplay->imageViewerModel()->display();

	update();
}

QOpenGLFramebufferObject *ImageRenderer::createFramebufferObject(
		const QSize &size)
{
	QOpenGLFramebufferObjectFormat format;
	format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
	format.setSamples(4);
	return new QOpenGLFramebufferObject(size, format);
}

/////////////
QQuickFramebufferObject::Renderer *ImageViewerDisplay::createRenderer() const
{
	return new ImageRenderer(this);
}

void ImageViewerDisplay::update() {}
}  // namespace processor
}  // namespace xray
}  // namespace sofa
