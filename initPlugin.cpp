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

#include <sofa/helper/system/config.h>
#include "initPlugin.h"

namespace sofaor
{
/**
 * \brief Data processing library
 *  provides 2D / volumic image, point cloud and sensors data processing algorithms to extract simulation-relevant data.
 */
namespace processor
{
// Here are just several convenient functions to help user to know what contains
// the plugin

extern "C" {
SOFA_PROCESSORPLUGIN_API void initExternalModule();
SOFA_PROCESSORPLUGIN_API const char* getModuleName();
SOFA_PROCESSORPLUGIN_API const char* getModuleVersion();
SOFA_PROCESSORPLUGIN_API const char* getModuleLicense();
SOFA_PROCESSORPLUGIN_API const char* getModuleDescription();
SOFA_PROCESSORPLUGIN_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
  }
}

const char* getModuleName() { return "ProcessOR"; }
const char* getModuleVersion() { return "0.1"; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription()
{
  return "ProcessOR's Base module";
}

const char* getModuleComponentList()
{
  return "";
}

}  // namespace processor
}  // namespace sofaor
////////// BEGIN CLASS LIST //////////

