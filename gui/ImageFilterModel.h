#ifndef IMAGEFILTERMODEL_H
#define IMAGEFILTERMODEL_H

#include <QObject>

#include "ProcessOR/initPlugin.h"
#include "common/ImageFilter.h"

#include <SofaQtQuickGUI/SofaComponent.h>
#include <SofaQtQuickGUI/SofaComponentList.h>
#include <SofaQtQuickGUI/SofaQtQuickGUI.h>

namespace sofaor
{
namespace processor
{
class SOFA_PROCESSOR_API ImageFilterModel : public QObject
{
  Q_OBJECT

 public:
  ImageFilterModel(QObject* parent = 0);

 public:
  Q_PROPERTY(sofa::qtquick::SofaComponent* sofaComponent READ sofaComponent
                 WRITE setSofaComponent NOTIFY sofaComponentChanged)
 public:
  sofa::qtquick::SofaComponent* sofaComponent() const;
  void setSofaComponent(sofa::qtquick::SofaComponent* sofaComponent);

  sofaor::processor::ImageFilter* imageFilter() const;
  void setImageFilter(sofaor::processor::ImageFilter* imageFilter)
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
  mutable sofaor::processor::ImageFilter*
      m_imageFilter;  // make it more abstract
};

class SOFA_PROCESSOR_API ImageFilterModelList : public QObject
{
  Q_OBJECT

 public:
  ImageFilterModelList(QObject* parent = 0);

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
  QList<sofaor::processor::ImageFilterModel*> m_imageFilterModelList;
};

}  // namespace processor
}  // namespace sofaor

#endif  // IMAGEFILTERMODEL_H
