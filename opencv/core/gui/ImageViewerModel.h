#ifndef IMAGEVIEWER_MODEL_H
#define IMAGEVIEWER_MODEL_H

#include <QObject>

#include <SofaQtQuickGUI/SofaComponent.h>
#include <SofaQtQuickGUI/SofaComponent.h>
#include <SofaQtQuickGUI/SofaComponentList.h>
#include <SofaQtQuickGUI/SofaQtQuickGUI.h>
#include "ProcessOR/initPlugin.h"
#include "core/ImageFilter.h"

namespace sofa
{
namespace OR
{
namespace processor
{
class SOFA_PROCESSORPLUGIN_API ImageViewerModel : public QObject
{
	Q_OBJECT

 public:
	ImageViewerModel(QObject* parent = 0);

 public:
	Q_PROPERTY(sofa::qtquick::SofaComponent* sofaComponent READ sofaComponent
								 WRITE setSofaComponent NOTIFY sofaComponentChanged)
 public:
	sofa::qtquick::SofaComponent* sofaComponent() const;
	void setSofaComponent(sofa::qtquick::SofaComponent* sofaComponent);

	sofa::OR::processor::ImageFilter* imageFilter() const;
	void setImageFilter(sofa::OR::processor::ImageFilter* imageFilter)
	{
		m_imageFilter = imageFilter;
	}

	void display();

 signals:
	void sofaComponentChanged();

 private slots:
	void handleSofaDataChange();

 private:
	sofa::qtquick::SofaComponent* m_sofaComponent;
	mutable sofa::OR::processor::ImageFilter*
			m_imageFilter;  // make it more abstract
};

class SOFA_PROCESSORPLUGIN_API ImageViewerModelList : public QObject
{
	Q_OBJECT

 public:
	ImageViewerModelList(QObject* parent = 0);

 public:
	Q_PROPERTY(
			sofa::qtquick::SofaComponentList sofaComponentList READ sofaComponentList
					WRITE setSofaComponentList NOTIFY sofaComponentListChanged)
 public:
	sofa::qtquick::SofaComponentList sofaComponentList() const;
	void setSofaComponentList(sofa::qtquick::SofaComponentList sofaComponentList);

	void display(int index);

 signals:
	void sofaComponentListChanged();

 private slots:
	void handleSofaDataChange();

 private:
	sofa::qtquick::SofaComponentList m_sofaComponentList;
	QList<sofa::OR::processor::ImageViewerModel*> m_imageModelList;
};

}  // namespace processor

}  // namespace OR

}  // namespace sofa

#endif  // IMAGEVIEWER_MODEL_H
