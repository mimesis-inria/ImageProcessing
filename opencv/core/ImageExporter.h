#ifndef SOFA_OR_PROCESSOR_IMAGEEXPORTER_H
#define SOFA_OR_PROCESSOR_IMAGEEXPORTER_H

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMat.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>

#include <opencv2/opencv.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
class ImageExporter : public common::ImplicitDataEngine
{
  Data<std::string> d_fileName;
  Data<common::cvMat> d_img;
	Data<unsigned> d_nSteps;
	Data<helper::OptionsGroup> d_exportType;
	Data<bool> d_activate;

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
  }

  void update()
  {
    ++m_stepCounter;

    cv::Mat img;

    if (d_img.getValue().type() == CV_32FC1)
    {
      msg_warning("ImageExporter::export()")
          << "CV_32F matrices will be normalized into a CV_8U matrix. Consider "
             "converting first to optimize performances";
      cv::normalize(d_img.getValue(), img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }

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
          cv::imwrite(std::to_string(m_stepCounter) + d_fileName.getValue(),
                      img, qualityType);
        }
        break;
      default:
        break;
    }
  }

  void cleanup()
  {
      std::cout << "cleanup called on exporter" << std::endl;
    if (d_exportType.getValue().getSelectedId() == 2)  // END
    {
      std::vector<int> qualityType;
      qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
      qualityType.push_back(90);

      cv::Mat img;
      if (d_img.getValue().type() == CV_32FC1)
      {
        msg_warning("ImageExporter::export()")
            << "CV_32F matrices will be normalized into a CV_8U matrix. Consider "
               "converting first to optimize performances";
        cv::normalize(d_img.getValue(), img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      }
        std::cout << "writing image" << std::endl;
      cv::imwrite(d_fileName.getValue(), img, qualityType);
    }
  }

 private:
  unsigned m_stepCounter;
};

SOFA_DECL_CLASS(ImageExporter)

int ImageExporterClass =
    core::RegisterObject(
        "component to export Opencv images as a file on your system")
        .add<ImageExporter>();

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_IMAGEEXPORTER_H