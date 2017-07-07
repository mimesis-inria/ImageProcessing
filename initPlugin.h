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

#ifndef PROCESSORPLUGIN_H
#define PROCESSORPLUGIN_H

#include <sofa/helper/system/config.h>

#define PROCESSORPLUGIN_MAJOR_VERSION \
  $ { PROCESSORPLUGIN_MAJOR_VERSION }
#define PROCESSORPLUGIN_MINOR_VERSION \
  $ { PROCESSORPLUGIN_MINOR_VERSION }
#ifdef SOFA_BUILD_PROCESSORPLUGIN
#define SOFA_TARGET PROCESSORPlugin
#define SOFA_PROCESSORPLUGIN_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_PROCESSORPLUGIN_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#endif  // PROCESSORPLUGIN_H
