#include "ImageViewerModel.h"

#include <SofaQtQuickGUI/SofaScene.h>

namespace sofa
{
namespace OR
{
namespace processor
{
ImageViewerModel::ImageViewerModel(QObject* parent)
		: QObject(parent), m_sofaComponent(0)
{
	connect(this, &ImageViewerModel::sofaComponentChanged, this,
					&ImageViewerModel::handleSofaDataChange);
}

qtquick::SofaComponent* ImageViewerModel::sofaComponent() const
{
	return m_sofaComponent;
}

void ImageViewerModel::setSofaComponent(qtquick::SofaComponent* sofaComponent)
{
	if (sofaComponent == m_sofaComponent) return;

	m_sofaComponent = sofaComponent;
	if (sofaComponent)
		m_sofaComponent = new qtquick::SofaComponent(*sofaComponent);

	sofaComponentChanged();
}

void ImageViewerModel::handleSofaDataChange()
{
	if (!m_sofaComponent) return;

	core::objectmodel::Base* baseComponent = m_sofaComponent->base();
	if (!baseComponent) return;

	// Here, list all potential renderer
	QString type = QString::fromStdString(baseComponent->getTypeName());
	if (0 == type.compare("ImageFilter"))
		setImageFilter(
				dynamic_cast<sofa::OR::processor::ImageFilter*>(baseComponent));
	else
		msg_error("ImageViewerModel") << "Type unknown";
}

sofa::OR::processor::ImageFilter* ImageViewerModel::imageFilter() const
{
	if (!m_sofaComponent)
	{
		m_imageFilter = 0;
	}
	else
	{
		const core::objectmodel::Base* baseComponent = m_sofaComponent->base();
		if (!baseComponent) m_imageFilter = 0;
	}

	return m_imageFilter;
}

void ImageViewerModel::display()
{
	if (this->imageFilter()) this->imageFilter()->drawImage();
}

////////

ImageViewerModelList::ImageViewerModelList(QObject* parent)
		: QObject(parent), m_sofaComponentList(NULL)
{
	connect(this, &ImageViewerModelList::sofaComponentListChanged, this,
					&ImageViewerModelList::handleSofaDataChange);
}

sofa::qtquick::SofaComponentList ImageViewerModelList::sofaComponentList() const
{
	return m_sofaComponentList;
}

void ImageViewerModelList::setSofaComponentList(
		sofa::qtquick::SofaComponentList sofaComponentList)
{
	m_sofaComponentList = sofaComponentList;

	sofaComponentListChanged();
}

void ImageViewerModelList::handleSofaDataChange()
{
	if (m_sofaComponentList.size() < 1) return;

	for (sofa::OR::processor::ImageViewerModel* imageModel : m_imageModelList)
		delete imageModel;

	m_imageModelList.clear();

	for (unsigned int i = 0; i < m_sofaComponentList.size(); i++)
	// for (qtquick::SofaComponent* sofaComponent : m_sofaComponentList)
	{
		// core::objectmodel::Base* baseComponent = sofaComponent->base();
		core::objectmodel::Base* baseComponent = m_sofaComponentList.at(i)->base();
		if (!baseComponent)
    {
			// Here, list all potential renderer
			QString type = QString::fromStdString(baseComponent->getTypeName());
			if (0 == type.compare("ImageFilter"))
			{
				sofa::OR::processor::ImageViewerModel* imageModel =
						new sofa::OR::processor::ImageViewerModel();
				m_imageModelList.push_back(imageModel);
				imageModel->setImageFilter(
						dynamic_cast<sofa::OR::processor::ImageFilter*>(baseComponent));
			}
			else
				msg_error("ImageViewerModelList") << "Type unknown";
    }
	}
}

void ImageViewerModelList::display(int index)
{
	if (m_sofaComponentList.size() <= index) return;

	sofa::OR::processor::ImageViewerModel* imageModel = m_imageModelList[index];

	if (imageModel) imageModel->display();
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
