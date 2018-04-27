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

#include "FrameExporter.h"

namespace sofacv
{
namespace common
{

FrameExporter::FrameExporter()
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

void FrameExporter::init()
{
  m_stepCounter = 0;
  addInput(&d_img);
  update();
}

void FrameExporter::Update()
{
  ++m_stepCounter;

  cv::Mat img;

  if (d_img.getValue().empty()) return;
  if (d_img.getValue().type() == CV_32FC1)
  {
    msg_warning("FrameExporter::export()")
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
        cv::imwrite(std::to_string(m_stepCounter) + "_" + d_fileName.getValue(),
                    img, qualityType);
      }
      break;
    default:
      break;
  }
}

void FrameExporter::cleanup()
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
      msg_warning("FrameExporter::export()")
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

} // namespace common
} // namespace sofacv
