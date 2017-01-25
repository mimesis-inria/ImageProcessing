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
      return int((val * 255.0) / max);
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
      return 255;
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
      double max = value_max._double - value_min._double;
      reinterpret_cast<Data<double>*>(data)->setValue((double(val) * max) /
                                                      255.0);
      break;
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

ImageFilter::ImageFilter()
    : d_in(initData(
          &d_in, common::cvMat(), "in",
          "Input image, that will undergo changes through the filter.")),
      d_out(
          initData(&d_out, "out", "Output image, holding the filter's result")),
      d_displayDebugWindow(initData(&d_displayDebugWindow, false, "Debug",
                                    "Display a debug window to see in live "
                                    "the changes applied to the filter")),
      m_outputImage(true),
      m_win_name(std::to_string(m_window_uid) + "_" + getClassName())
{
  f_listening.setValue(true);
  m_window_uid++;
}

ImageFilter::~ImageFilter() {}
void ImageFilter::getInputFromContext()
{
  ImageFilter* lastFilter = this->getContext()->get<ImageFilter>();
  if (lastFilter && lastFilter != this)
  {
    d_in.setParent(&lastFilter->d_out,
                   "@" + lastFilter->getPathName() + ".out");
    msg_info(getClassName() + "::init()")
        << "ImageFilter Note: no input image given to the "
           "filter. Linking to last filter's "
           "output image";
  }
  else
  {
    acquisitor::BaseFrameGrabber* grabber =
        this->getContext()->get<acquisitor::BaseFrameGrabber>();
    if (grabber)
    {
      d_in.setParent(&grabber->d_frame,
                     "@" + grabber->getPathName() + ".frame");
      msg_warning(getClassName() + "::init()")
          << "ImageFilter: No input image given to the "
             "filter. Linking to last grabber's"
             "output frame";
    }
    else
      msg_error(getClassName() + "::init()")
          << "ImageFilter Error: No Previous ImageFilter nor "
             "ImageGrabber found in sceneGraph.";
  }
}

void ImageFilter::init()
{
  if (!d_in.isSet()) getInputFromContext();

  addInput(&d_in);
  addOutput(&d_out);
  setDirtyValue();
}

void ImageFilter::update()
{
  updateAllInputsIfDirty();
  cleanDirty();
  if (!f_listening.getValue())
  {
    // filter inactive, out = in
    d_out.setValue(d_in.getValue());
    d_out.endEdit();
    d_out.setDirtyOutputs();
  }
  cv::Mat empty = d_in.getValue().zeros(
      d_in.getValue().rows, d_in.getValue().cols, d_in.getValue().type());
  applyFilter(d_in.getValue(), empty);
  if (!m_outputImage)
  {
    d_out.setValue(d_in.getValue());
    d_out.endEdit();
  }
  else
  {
    d_out.setValue(empty);
    d_out.endEdit();
  }
  d_out.setDirtyOutputs();
  if (d_displayDebugWindow.getValue()) cv::imshow(m_win_name, empty);
}

void ImageFilter::reinit()
{
  drawDebug();
  if (m_outputImage)
  {
    m_debugImage.copyTo(*d_out.beginEdit());
    d_out.endEdit();
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
  applyFilter(d_in.getValue(), m_debugImage, true);
  if (m_debugImage.empty()) return;
  cv::imshow(m_win_name, m_debugImage);
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
  m_params.push_back(Holder(Holder::DOUBLE, data, min, max, step));
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
