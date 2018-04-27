#ifndef SOFACV_GUI_IMAGEFILTERMODEL_H
#define SOFACV_GUI_IMAGEFILTERMODEL_H

#include <QObject>

#include "ImageProcessingPlugin.h"
#include "common/ImageFilter.h"

#include <SofaQtQuickGUI/SofaComponent.h>
#include <SofaQtQuickGUI/SofaComponentList.h>
#include <SofaQtQuickGUI/SofaQtQuickGUI.h>

namespace sofacv
{
namespace gui
{
class SOFA_IMAGEPROCESSING_API ImageFilterModel : public QObject
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

  sofacv::common::ImageFilter* imageFilter() const;
  void setImageFilter(sofacv::common::ImageFilter* imageFilter);

  void display();

 signals:
  void sofaComponentChanged();

 private slots:
  void handleSofaDataChange();

 private:
  sofa::qtquick::SofaComponent* m_sofaComponent;
  mutable sofacv::common::ImageFilter*
      m_imageFilter;  // make it more abstract
};

class SOFA_IMAGEPROCESSING_API ImageFilterModelList : public QObject
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
  QList<sofacv::gui::ImageFilterModel*> m_imageFilterModelList;
};

}  // namespace gui
}  // namespace sofacv

#endif  // SOFACV_GUI_IMAGEFILTERMODEL_H
