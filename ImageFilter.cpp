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
    case INT:
    {
      int max = value_max._int - value_min._int;
      int val = reinterpret_cast<Data<int>*>(data)->getValue() - value_min._int;
      return (val * 255) / max;
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

void ImageFilter::Holder::setDataValue(int val)
{
  switch (type)
  {
    case BOOL:
      reinterpret_cast<Data<bool>*>(data)->setValue((val == 1) ? (true)
                                                               : (false));
      break;
    case INT:
    {
      int max = value_max._int - value_min._int;
      reinterpret_cast<Data<int>*>(data)->setValue((val * max) / 255);
    }
    break;
    case DOUBLE:
      double max = value_max._double - value_min._double;
      reinterpret_cast<Data<double>*>(data)->setValue((double(val) * max) /
                                                      255.0);
      break;
  }
}

void ImageFilter::Holder::drawDebug()
{
  dynamic_cast<ImageFilter*>(
      reinterpret_cast<core::objectmodel::BaseData*>(data)->getOwner())
      ->drawDebug();
}

void ImageFilter::callback(int val, void* holder)
{
  std::cout << "val= " << val << std::endl;
  reinterpret_cast<Holder*>(holder)->setDataValue(val);
  reinterpret_cast<Holder*>(holder)->drawDebug();
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
    d_in.setParent(&lastFilter->d_out, "out");
    msg_info("ImageFilter::init()") << "Note: no input image given to the "
                                       "filter. Linking to last filter's "
                                       "output image";
  }
  else
  {
    acquisitor::BaseFrameGrabber* grabber =
        this->getContext()->get<acquisitor::BaseFrameGrabber>();
    if (grabber)
    {
      d_in.setParent(&grabber->d_frame, "frame");
      msg_warning("ImageFilter::init()") << "No input image given to the "
                                            "filter. Linking to last grabber's"
                                            "output frame";
    }
    else
      msg_error("ImageFilter::init()") << "Error: No Previous ImageFilter nor "
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
    d_in.getValue().copyTo(*d_out.beginWriteOnly());
    d_out.endEdit();
    return;
  }
  applyFilter(d_in.getValue(), *d_out.beginEdit());
  d_out.endEdit();
  d_out.setDirtyOutputs();
}

void ImageFilter::reinit()
{
  drawDebug();
  m_debugImage.copyTo(*d_out.beginEdit());
  d_out.endEdit();
}

void ImageFilter::drawDebug()
{
  if (!d_displayDebugWindow.getValue())
  {
    cv::destroyWindow(m_win_name);
    return;
  }
  cv::namedWindow(m_win_name, CV_WINDOW_AUTOSIZE);
  for (Holder& h : m_params)
  {
    switch (h.type)
    {
      case Holder::BOOL:
        cv::createTrackbar(h.data->getName(), m_win_name, 0, 1,
                           &ImageFilter::callback, &m_params.back());
        cv::setTrackbarPos(h.data->getName(), m_win_name,
                           m_params.back().getTrackbarRangedValue());
        break;
      case Holder::INT:
        cv::createTrackbar(h.data->getName(), m_win_name, 0, 255,
                           &ImageFilter::callback, &m_params.back());
        cv::setTrackbarPos(h.data->getName(), m_win_name,
                           m_params.back().getTrackbarRangedValue());
        break;
      case Holder::DOUBLE:
        cv::createTrackbar(h.data->getName(), m_win_name, 0, 255,
                           &ImageFilter::callback, &m_params.back());
        cv::setTrackbarPos(h.data->getName(), m_win_name,
                           m_params.back().getTrackbarRangedValue());
        break;
    }
  }
  applyFilter(d_in.getValue(), m_debugImage);
  if (m_debugImage.empty()) return;
  cv::imshow(m_win_name, m_debugImage);
}

void ImageFilter::registerData(Data<bool>* data)
{
  m_params.push_back(Holder(data));
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

}  // namespace processor
}  // namespace OR
}  // namespace sofa
