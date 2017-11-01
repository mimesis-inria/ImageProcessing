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

#ifndef SOFA_OR_PROCESSOR_IMAGEEXPORTER_H
#define SOFA_OR_PROCESSOR_IMAGEEXPORTER_H

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
 * @brief The ImageExporter class
 *
 * Export an input image (img) into [fileName] either at BEGIN, END or at a
 * specific frequency of the simulation (STEP).
 * Can be activated / deactivated manually with [active]
 */
class ImageExporter : public common::ImplicitDataEngine
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
  SOFA_CLASS(ImageExporter, common::ImplicitDataEngine);

  ImageExporter()
      : d_fileName(initData(&d_fileName, "fileName", "output image file name")),
        d_img(initData(&d_img, "img", "image to export")),
        d_nSteps(initData(&d_nSteps, (unsigned)0, "nSteps",
                          "number of steps between each export (0 means no "
                          "export during animation")),
        d_exportType(initData(&d_exportType, "exportType",
                              "specify when export should happen (BEGIN, END, "
                              "STEP). if STEP, specify export frequency with "
                              "nSteps. Default is END")),
        d_activate(initData(&d_activate, true, "active",
                            "if false, nothing will be exported")),
        m_stepCounter(0)
  {
    sofa::helper::OptionsGroup* t = d_exportType.beginEdit();
    t->setNames(3, "BEGIN", "END", "STEP");
    t->setSelectedItem("END");
    d_exportType.endEdit();
  }

  ~ImageExporter() {}
  void init()
  {
    m_stepCounter = 0;
    addInput(&d_img);
    update();
  }

  void update()
  {
    ++m_stepCounter;

    cv::Mat img;

    if (d_img.getValue().empty())
        return;
    if (d_img.getValue().type() == CV_32FC1)
    {
      msg_warning("ImageExporter::export()")
          << "CV_32F matrices will be normalized into a CV_8U matrix. Consider "
             "converting first to optimize performances";
      cv::normalize(d_img.getValue(), img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }
    else
      img = d_img.getValue();

    std::vector<int> qualityType;
    qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
    qualityType.push_back(90);

    switch (d_exportType.getValue().getSelectedId())
    {
      case 0:  // BEGIN
        if (m_stepCounter == 1)
        {
          cv::imwrite(d_fileName.getValue(), img, qualityType);
        }
        break;
      case 2:  // STEP
        if (m_stepCounter % d_nSteps.getValue() == 0)
        {
          cv::imwrite(
              std::to_string(m_stepCounter) + "_" + d_fileName.getValue(), img,
              qualityType);
        }
        break;
      default:
        break;
    }
  }

  void cleanup()
  {
    std::cout << "cleanup called on exporter" << std::endl;
    if (d_exportType.getValue().getSelectedId() == 1)  // END
    {
      std::vector<int> qualityType;
      qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
      qualityType.push_back(90);

      cv::Mat img;
      if (d_img.getValue().type() == CV_32FC1)
      {
        msg_warning("ImageExporter::export()")
            << "CV_32F matrices will be normalized into a CV_8U matrix. "
               "Consider "
               "converting first to optimize performances";
        cv::normalize(d_img.getValue(), img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      }
      else
        img = d_img.getValue();
      cv::imwrite(d_fileName.getValue(), img, qualityType);
    }
  }

 private:
  unsigned m_stepCounter;
};

SOFA_DECL_CLASS(ImageExporter)

int ImageExporterClass =
    sofa::core::RegisterObject(
        "component to export Opencv images as a file on your system")
        .add<ImageExporter>();

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_IMAGEEXPORTER_H
