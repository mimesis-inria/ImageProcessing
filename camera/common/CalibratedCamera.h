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

#ifndef SOFACV_CAM_CALIBRATEDCAMERA_H
#define SOFACV_CAM_CALIBRATEDCAMERA_H

#include "ImageProcessingPlugin.h"

#include "CameraSettings.h"

#include <SofaCV/SofaCV.h>

#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/visual/DrawToolGL.h>
#include <sofa/core/visual/VisualManager.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/gl/Transformation.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glu.h>

#include <opencv2/opencv.hpp>

namespace sofacv
{
namespace cam
{
/**
 * @brief The CalibratedCamera class
 *
 * This component gets / sets OpenGL parameters from / to a linked
 * CameraSettings component, and modifies the OpenGL view in SOFA
 */
class SOFA_IMAGEPROCESSING_API CalibratedCamera : public ImplicitDataEngine,
                         public sofa::core::visual::VisualManager
{
  typedef sofa::core::objectmodel::SingleLink<
      CalibratedCamera, CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

  typedef typename sofa::defaulttype::Vector3 Vector3;

 public:
  SOFA_CLASS2(CalibratedCamera, ImplicitDataEngine,
              sofa::core::visual::VisualManager);

  CalibratedCamera();

  virtual ~CalibratedCamera() override {}
  void init() override;
  void reinit() override {}
  void Update() override;

  /// sets the correct GL params for drawing and displays the camera gizmo if
  /// drawGizmo=true
  void preDrawScene(sofa::core::visual::VisualParams* vparams) override;

  /// Restores initial GL parameters
  void postDrawScene(sofa::core::visual::VisualParams* /*vp*/) override;

  /// Overriden implementation of ImplicitDataEngine's handleEvent
  /// Adds a hook on the C-u keyboard shortcut to set GL params in
  /// CameraSettings
  virtual void handleEvent(sofa::core::objectmodel::Event* e) override;

  void computeBBox(const sofa::core::ExecParams* params, bool) override;

  CamSettings l_cam;           ///< The linked CameraSettings component
  sofa::Data<bool> d_freeCam;  ///< locks / unlocks the modelview in OpenGL
  sofa::Data<bool>
      d_freeProj;  ///< set / unset CameraSettings intrinsic params in OpenGL
  sofa::Data<bool> d_drawGizmo;  ///< draws / hides the camera gizmo
  sofa::Data<bool> d_captureFrame;  ///< captures camera's viewport as cvMat
  sofa::Data<cvMat> d_img;  ///< captured camera frame

 private:
  bool m_storeMatrices;
};

}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CALIBRATEDCAMERA_H
