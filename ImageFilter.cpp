#include "ImageFilter.h"
#include <AcquisitOR/BaseFrameGrabber.h>
#include <opencv2/highgui.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
int ImageFilter::Holder::getTrackbarRangedValue()
{
  switch (type)
  {
    case BOOL:
      return reinterpret_cast<Data<bool>*>(data)->getValue();
    case OPTIONSGROUP:
    {
      return int(reinterpret_cast<Data<helper::OptionsGroup>*>(data)
                     ->getValue()
                     .getSelectedId());
    }
    case INT:
    {
      return reinterpret_cast<Data<int>*>(data)->getValue() - value_min._int;
    }
    case DOUBLE:
    {
      double max = value_max._double - value_min._double;
      double val =
          reinterpret_cast<Data<double>*>(data)->getValue() - value_min._double;
      return int((val * (value_max._double / step._double)) / max);
    }
    case FLOAT:
    {
      float max = value_max._float - value_min._float;
      float val =
          reinterpret_cast<Data<float>*>(data)->getValue() - value_min._float;
      return int((val * (value_max._float / step._float)) / max);
    }
  }
  return 0;
}

int ImageFilter::Holder::getTrackbarMaxValue()
{
  switch (type)
  {
    case BOOL:
      return 1;
    case OPTIONSGROUP:
    case INT:
      return value_max._int - value_min._int;
    case DOUBLE:
      return int(value_max._double / step._double);
    case FLOAT:
      return int(value_max._float / step._float);
  }
  return 0;
}

void ImageFilter::Holder::setDataValue(int val)
{
  switch (type)
  {
    case BOOL:
      reinterpret_cast<Data<bool>*>(data)->setValue((val == 1) ? (true)
                                                               : (false));
      break;
    case INT:
      reinterpret_cast<Data<int>*>(data)->setValue(val + value_min._int);
      break;
    case OPTIONSGROUP:
      reinterpret_cast<Data<helper::OptionsGroup>*>(data)
          ->beginEdit()
          ->setSelectedItem(unsigned(val));
      break;
    case DOUBLE:
    {
      double max = value_max._double - value_min._double;
      reinterpret_cast<Data<double>*>(data)->setValue(
          (double(val) * max) / (value_max._double / step._double));
      break;
    }
    case FLOAT:
    {
      float max = value_max._float - value_min._float;
      reinterpret_cast<Data<float>*>(data)->setValue(
          (float(val) * max) / (value_max._float / step._float));
      break;
    }
  }
}

void ImageFilter::Holder::refresh()
{
  dynamic_cast<ImageFilter*>(
      reinterpret_cast<core::objectmodel::BaseData*>(data)->getOwner())
      ->refreshDebugWindow();
}

void ImageFilter::callback(int val, void* holder)
{
  if (reinterpret_cast<Holder*>(holder)->getTrackbarRangedValue() != val)
  {
    reinterpret_cast<Holder*>(holder)->setDataValue(val);
    reinterpret_cast<Holder*>(holder)->refresh();
  }
}

unsigned ImageFilter::m_window_uid = 0;

ImageFilter::ImageFilter(bool outputImage)
    : d_img(initData(
          &d_img, common::cvMat(), "img",
          "Input image, that will undergo changes through the filter.")),
      d_img_out(initData(&d_img_out, "img_out",
                         "Output image, holding the filter's result")),
      d_displayDebugWindow(initData(&d_displayDebugWindow, false, "Debug",
                                    "Display a debug window to see in live "
                                    "the changes applied to the filter")),
      m_outputImage(outputImage),
      m_win_name(std::to_string(m_window_uid) + "_" + getClassName())
{
  addAlias(&d_img_out, "img1_out");
  f_listening.setValue(true);
  m_window_uid++;
}

ImageFilter::~ImageFilter() {}
void ImageFilter::init()
{
  std::cout << getClassName() << "init" << std::endl;

  bindInputData(&d_img);
  addOutput(&d_img_out);
  setDirtyValue();
}

void ImageFilter::update()
{
  updateAllInputsIfDirty();
  cleanDirty();
  if (!f_listening.getValue())
  {
    // filter inactive, out = in
    d_img_out.setValue(d_img.getValue());
    d_img_out.endEdit();
    d_img_out.setDirtyOutputs();
  }
  cv::Mat empty = d_img.getValue().zeros(
      d_img.getValue().rows, d_img.getValue().cols, d_img.getValue().type());
  applyFilter(d_img.getValue(), empty);
  if (!m_outputImage)
  {
    d_img_out.setValue(d_img.getValue());
    d_img_out.endEdit();
  }
  else
  {
    d_img_out.setValue(empty);
    d_img_out.endEdit();
  }
  d_img_out.setDirtyOutputs();
  if (d_displayDebugWindow.getValue() && !empty.empty())
  {
    cv::imshow(m_win_name, empty);
    cv::waitKey(1);
  }
}

void ImageFilter::reinit()
{
  drawDebug();
  if (m_outputImage)
  {
    m_debugImage.copyTo(*d_img_out.beginEdit());
    d_img_out.endEdit();
  }
}

bool ImageFilter::reinitDebugWindow()
{
  if (!d_displayDebugWindow.getValue())
  {
    cv::destroyWindow(m_win_name);
    return false;
  }

  cv::namedWindow(m_win_name, CV_WINDOW_AUTOSIZE);
  for (Holder& h : m_params)
  {
    cv::createTrackbar(h.data->getName(), m_win_name, 0,
                       h.getTrackbarMaxValue(), &ImageFilter::callback, &h);
    cv::setTrackbarPos(h.data->getName(), m_win_name,
                       h.getTrackbarRangedValue());
  }
  return true;
}

void ImageFilter::refreshDebugWindow()
{
  applyFilter(d_img.getValue(), m_debugImage, true);
  if (m_debugImage.empty()) return;

  cv::imshow(m_win_name, m_debugImage);
  cv::waitKey(1);
}

void ImageFilter::drawDebug()
{
  if (reinitDebugWindow()) refreshDebugWindow();
}

void ImageFilter::unregisterAllData() { m_params.clear(); }
void ImageFilter::registerData(Data<bool>* data, int min, int max, int step)
{
  m_params.push_back(Holder(Holder::BOOL, data, min, max, step));
}

void ImageFilter::registerData(Data<helper::OptionsGroup>* data, int min,
                               int max, int step)
{
  m_params.push_back(Holder(Holder::OPTIONSGROUP, data, min, max, step));
}

void ImageFilter::registerData(Data<int>* data, int min, int max, int step)
{
  m_params.push_back(Holder(Holder::INT, data, min, max, step));
}
void ImageFilter::registerData(Data<double>* data, double min, double max,
                               double step)
{
  m_params.push_back(Holder(Holder::DOUBLE, data, min, max, step));
}
void ImageFilter::registerData(Data<float>* data, float min, float max,
                               float step)
{
  m_params.push_back(Holder(Holder::FLOAT, data, min, max, step));
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
