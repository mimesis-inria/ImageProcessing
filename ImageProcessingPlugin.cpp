#include "ImageProcessingPlugin.h"
#include <sofa/helper/system/config.h>

#ifdef QT_PLUGIN
#include <gui/ImageFilterDisplay.h>
#include <gui/ImageFilterModel.h>
#include <QApplication>
#include <QDebug>
#include <QQuickPaintedItem>

const int versionMajor = 1;
const int versionMinor = 0;

static void initResources() { Q_INIT_RESOURCE(ImageProcessing_qml); }
#endif  // QT_PLUGIN

namespace sofacv
{
/**
 * \brief Data processing library
 *  provides 2D / volumic image, point cloud and sensors data processing
 * algorithms to extract simulation-relevant data.
 */

namespace features
{
}  // namespace features

/**
 * \brief Image processing filters
 *
 * implements ImageFilter, and provides image filtering components such as
 * Sobel, Canny etc.
 */
namespace imgproc
{
}  // namespace imgproc

// Here are just several convenient functions to help user to know what contains
// the plugin

extern "C" {
SOFA_IMAGEPROCESSING_API void initExternalModule();
SOFA_IMAGEPROCESSING_API const char* getModuleName();
SOFA_IMAGEPROCESSING_API const char* getModuleVersion();
SOFA_IMAGEPROCESSING_API const char* getModuleLicense();
SOFA_IMAGEPROCESSING_API const char* getModuleDescription();
SOFA_IMAGEPROCESSING_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
#ifdef QT_PLUGIN
    initResources();

    qmlRegisterType<sofacv::gui::ImageFilterDisplay>(
        "ImageFilterDisplay", versionMajor, versionMinor, "ImageFilterDisplay");
    qmlRegisterType<sofacv::gui::ImageFilterModel>(
        "ImageFilterModel", versionMajor, versionMinor, "ImageFilterModel");
    qmlRegisterType<sofacv::gui::ImageFilterModelList>(
        "ImageFilterModelList", versionMajor, versionMinor,
        "ImageFilterModelList");
#endif  // QT_PLUGIN
  }
}

const char* getModuleName() { return "ImageProcessing"; }
const char* getModuleVersion() { return ImageProcessing_VERSION; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription()
{
  return "SofaCV's Image processing module";
}

const char* getModuleComponentList() { return ""; }

}  // namespace sofacv
////////// BEGIN CLASS LIST //////////
