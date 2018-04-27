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

#ifndef SOFACV_COMMON_IMAGEFILTER_H
#define SOFACV_COMMON_IMAGEFILTER_H

#ifdef SOFACV_NEW_GUI_ENABLED
#include "ImageFilter_newGUI.h"
#else
#include "ImageFilter_oldGUI.h"
#endif  // SOFACV_NEW_GUI_ENABLED

//int ImageFilterClass = sofa::core::RegisterObject("Base class for SofaOR's image processing filters")
//        .add<sofaor::processor::ImageFilter>();

#endif  // SOFACV_COMMON_IMAGEFILTER_H
