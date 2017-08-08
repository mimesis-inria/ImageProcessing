/******************************************************************************
*       SOFAOR, SOFA plugin for the Operating Room, development version       *
*                        (c) 2017 INRIA, MIMESIS Team                         *
*                                                                             *
* This program is a free software; you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 1.0 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Bruno Marques and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact-mimesis@inria.fr                               *
******************************************************************************/

#include "initPlugin.h"
#include <sofa/helper/system/config.h>

#ifdef QT_PLUGIN
#include <gui/ImageFilterDisplay.h>
#include <gui/ImageFilterModel.h>
#include <QApplication>
#include <QDebug>
#include <QQuickPaintedItem>

const int versionMajor = 1;
const int versionMinor = 0;

static void initResources() { Q_INIT_RESOURCE(ProcessOR_qml); }
#endif  // QT_PLUGIN

namespace sofaor
{
/**
 * \brief Data processing library
 *  provides 2D / volumic image, point cloud and sensors data processing
 * algorithms to extract simulation-relevant data.
 */
namespace processor
{
/**
 * \brief Feature detection & matching
 *
 * provides 2D feature detection, extraction and matching algorithms, mainly
 * from OpenCV's feature2D and xfeature2D modules.
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
SOFA_PROCESSOR_API void initExternalModule();
SOFA_PROCESSOR_API const char* getModuleName();
SOFA_PROCESSOR_API const char* getModuleVersion();
SOFA_PROCESSOR_API const char* getModuleLicense();
SOFA_PROCESSOR_API const char* getModuleDescription();
SOFA_PROCESSOR_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
#ifdef QT_PLUGIN
    initResources();

    qmlRegisterType<sofaor::processor::ImageFilterDisplay>(
        "ImageFilterDisplay", versionMajor, versionMinor, "ImageFilterDisplay");
    qmlRegisterType<sofaor::processor::ImageFilterModel>(
        "ImageFilterModel", versionMajor, versionMinor, "ImageFilterModel");
    qmlRegisterType<sofaor::processor::ImageFilterModelList>(
        "ImageFilterModelList", versionMajor, versionMinor,
        "ImageFilterModelList");
#endif  // QT_PLUGIN
  }
}

const char* getModuleName() { return "ProcessOR"; }
const char* getModuleVersion() { return "0.1"; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription() { return "ProcessOR's Base module"; }

const char* getModuleComponentList()
{
    return "";
}

}  // namespace processor
}  // namespace sofaor
////////// BEGIN CLASS LIST //////////
