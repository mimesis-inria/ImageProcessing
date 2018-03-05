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

#ifndef SOFA_OR_PROCESSOR_FRAMEEXPORTER_H
#define SOFA_OR_PROCESSOR_FRAMEEXPORTER_H

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMat.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofaor
{
namespace processor
{
/**
 * @brief The FrameExporter class
 *
 * Export an input image (img) into [fileName] either at BEGIN, END or at a
 * specific frequency of the simulation (STEP).
 * Can be activated / deactivated manually with [active]
 */
class FrameExporter : public common::ImplicitDataEngine
{
  sofa::Data<std::string> d_fileName;  ///< name of the file to write into
  sofa::Data<common::cvMat> d_img;     ///< [INPUT] image to write in fileName
  sofa::Data<unsigned>
      d_nSteps;  ///< if exportType == STEP, frequency at which to export
  sofa::Data<sofa::helper::OptionsGroup> d_exportType;  ///< whether the image
                                                        /// should be written at
  /// begin or end of the
  /// simulation, or
  /// repeatedly every N
  /// steps
  sofa::Data<bool>
      d_activate;  ///< set to false to manually activate through GUI

 public:
  SOFA_CLASS(FrameExporter, common::ImplicitDataEngine);

  FrameExporter();

  ~FrameExporter() {}
  void init() override;

  void reinit() override {}

  void Update() override;

  void cleanup() override;

 private:
  unsigned m_stepCounter;
};

SOFA_DECL_CLASS(FrameExporter)

int FrameExporterClass =
    sofa::core::RegisterObject(
        "component to export Opencv images as a file on your system")
        .add<FrameExporter>();

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_FRAMEEXPORTER_H
