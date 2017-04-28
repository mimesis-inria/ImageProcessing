#ifndef IMAGEVIEWER_DISPLAY_H
#define IMAGEVIEWER_DISPLAY_H

#include <QObject>

#include "ImageViewerModel.h"
#include <math.h>
#include "core/ImageFilter.h"

#include <SofaQtQuickGUI/SofaComponent.h>
#include <SofaQtQuickGUI/SofaQtQuickGUI.h>

#include <QtQuick/QQuickFramebufferObject>

namespace sofa
{
namespace OR
{
namespace processor
{
class ImageViewerDisplay;

class SOFA_PROCESSORPLUGIN_API ImageRenderer
		: public QQuickFramebufferObject::Renderer
{
 public:
	ImageRenderer(const sofa::OR::processor::ImageViewerDisplay* imageViewerDisplay);
	void render();
	QOpenGLFramebufferObject* createFramebufferObject(const QSize& size);

 private:
	const sofa::OR::processor::ImageViewerDisplay* m_imageViewerDisplay;
};

class SOFA_PROCESSORPLUGIN_API ImageViewerDisplay : public QQuickFramebufferObject
{
	Q_OBJECT

	Q_PROPERTY(sofa::OR::processor::ImageViewerModel* imageViewerModel READ imageViewerModel
								 WRITE setImageViewerModel)  // NOTIFY imageViewerModelChanged)
 public:
	Renderer* createRenderer() const;

	sofa::OR::processor::ImageViewerModel* imageViewerModel() const
	{
		return m_imageViewerModel;
	}

 public slots:
	void update();

 protected:
	void setImageViewerModel(sofa::OR::processor::ImageViewerModel* imageViewerModel)
	{
		m_imageViewerModel = imageViewerModel;
	}

 private:
	sofa::OR::processor::ImageViewerModel* m_imageViewerModel;
};

}  // namespace proccessor
}  // namespace OR
}  // namespace sofa

#endif  // IMAGEVIEWER_DISPLAY_H
