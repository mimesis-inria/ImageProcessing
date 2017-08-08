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

#include "ImageFilter.h"
#include <AcquisitOR/BaseFrameGrabber.h>
#include <opencv2/highgui.hpp>
#include "DataSliderMgr.inl"

#include <sofa/core/visual/DrawToolGL.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>

namespace sofaor
{
namespace processor
{
SOFA_DECL_CLASS(ImageFilter)

int ImageFilterClass =
    sofa::core::RegisterObject("Abstract mother class for introspection")
        .add<ImageFilter>();

unsigned ImageFilter::m_window_uid = 0;

ImageFilter::ImageFilter()
    : d_img(initData(
          &d_img, common::cvMat(), "img",
          "Input image, that will undergo changes through the filter.", false)),
      d_img_out(initData(&d_img_out, "img_out",
                         "Output image, holding the filter's result", false)),
      d_isActive(initData(&d_isActive, true, "isActive",
                          "if false, does not call applyFilter")),
      d_outputImage(
          initData(&d_outputImage, true, "outputImage",
                   "whether or not the computed image will be set in img_out"))
#ifndef SOFAOR_NEW_GUI_ENABLED
      ,
      d_displayDebugWindow(initData(&d_displayDebugWindow, false, "Debug",
                                    "Display a debug window to see in live "
                                    "the changes applied to the filter")),
      m_win_name(std::to_string(m_window_uid) + "_" + getClassName())
#endif  // SOFAOR_NEW_GUI_ENABLED
{
  addAlias(&d_img_out, "img1_out");
  f_listening.setValue(true);
#ifndef SOFAOR_NEW_GUI_ENABLED
  m_window_uid++;
#endif  // SOFAOR_NEW_GUI_ENABLED
}

ImageFilter::~ImageFilter() {}
void ImageFilter::init()
{
  if (getClassName() == "ImageFilter")
    msg_error(getName())
        << "Cannot instantiate an abstract component of type ImageFilter";
#ifndef SOFAOR_NEW_GUI_ENABLED
  m_displayDebugDataTracker.trackData(d_displayDebugWindow);
#endif  // SOFAOR_NEW_GUI_ENABLED
  addInput(&d_img);
  addOutput(&d_img_out);
}

void ImageFilter::update()
{
  sofa::helper::AdvancedTimer::stepBegin("Image Filters");

  if (!d_isActive.getValue())
  {
    // filter inactive, out = in
    d_img_out.setValue(d_img.getValue());

#ifndef SOFAOR_NEW_GUI_ENABLED
    if (d_displayDebugWindow.getValue())
    {
      cv::imshow(m_win_name, d_img_out.getValue());
      cv::waitKey(1);
    }
#endif  // SOFAOR_NEW_GUI_ENABLED
    sofa::helper::AdvancedTimer::stepEnd("Image Filters");
    return;
  }
  m_debugImage = d_img.getValue().zeros(
      d_img.getValue().rows, d_img.getValue().cols, d_img.getValue().type());
  applyFilter(d_img.getValue(), m_debugImage);
  if (!d_outputImage.getValue())
  {
    d_img_out.setValue(d_img.getValue());
  }
  else
  {
    d_img_out.setValue(m_debugImage);
  }
#ifndef SOFAOR_NEW_GUI_ENABLED
  if (d_displayDebugWindow.getValue() && !m_debugImage.empty())
  {
    cv::imshow(m_win_name, m_debugImage);
    cv::waitKey(1);
  }
#endif  // SOFAOR_NEW_GUI_ENABLED
  sofa::helper::AdvancedTimer::stepEnd(("Image Filters"));
}

void ImageFilter::reinit()
{
  if (m_displayDebugDataTracker.isDirty() && d_isActive.getValue())
    refreshDebugWindow();

  // to set needsRefresh to true
  ImplicitDataEngine::reinit();
}

void ImageFilter::bindGlTexture(const std::string& imageString)
{
  glEnable(GL_TEXTURE_2D);  // enable the texture
  glDisable(GL_LIGHTING);   // disable the light

  glBindTexture(GL_TEXTURE_2D, 0);  // texture bind

  unsigned internalFormat = GL_RGB;
  unsigned format = GL_BGR_EXT;
  unsigned type = GL_UNSIGNED_BYTE;

  switch (m_debugImage.channels())
  {
    case 1:  // grayscale
      internalFormat = GL_LUMINANCE;
      format = GL_RED;
      break;
    case 3:  // RGB / BGR
      internalFormat = GL_RGB;

      format = GL_BGR_EXT;
      break;
    case 4:  // RGBA / BGRA
      internalFormat = GL_RGBA;
      format = GL_BGRA_EXT;
      break;
  }
  switch (m_debugImage.type())
  {
    case CV_8U:
      type = GL_UNSIGNED_BYTE;
      break;
    case CV_8S:
      type = GL_BYTE;
      break;
    case CV_16U:
      type = GL_UNSIGNED_SHORT;
    case CV_16S:
      type = GL_SHORT;
      break;
    case CV_32S:
      type = GL_INT;
      break;
    case CV_32F:
      type = GL_FLOAT;
      break;
    default:
      type = GL_UNSIGNED_BYTE;
      break;
  }
  glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, m_debugImage.cols,
               m_debugImage.rows, 0, format, type, imageString.c_str());
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

#ifdef SOFAOR_NEW_GUI_ENABLED
void ImageFilter::drawFullFrame()
{
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -1, 1);  // orthogonal view
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  std::stringstream imageString;
  imageString.write((const char*)m_debugImage.data,
                    m_debugImage.total() * m_debugImage.elemSize());

  bindGlTexture(imageString.str());

  glBegin(GL_QUADS);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glTexCoord2f(0, 0);
  glVertex2f(0, 0);
  glTexCoord2f(1, 0);
  glVertex2f(1, 0);
  glTexCoord2f(1, 1);
  glVertex2f(1, 1);
  glTexCoord2f(0, 1);
  glVertex2f(0, 1);
  glEnd();

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);     // enable light
  glDisable(GL_TEXTURE_2D);  // disable texture 2D
                             // glDepthMask (GL_TRUE);		// enable zBuffer
  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
}
#endif  // SOFAOR_NEW_GUI_ENABLED

#ifndef SOFAOR_NEW_GUI_ENABLED
void ImageFilter::reinitDebugWindow()
{
  if (!d_displayDebugWindow.getValue())
  {
    if (m_isMouseCallbackActive) cv::setMouseCallback(m_win_name, NULL, NULL);
    cv::destroyWindow(m_win_name);
    return;
  }

  // WITH QT:
  //  cv::namedWindow(m_win_name,
  //                  CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  cv::namedWindow(m_win_name, CV_WINDOW_AUTOSIZE);
  for (DSM* dmgr : m_params) dmgr->createSlider(m_win_name);
  if (m_isMouseCallbackActive)
    cv::setMouseCallback(m_win_name, &ImageFilter::_mouseCallback, this);
}
#endif  // SOFAOR_NEW_GUI_ENABLED

void ImageFilter::refreshDebugWindow()
{
#ifndef SOFAOR_NEW_GUI_ENABLED
  reinitDebugWindow();
  if (!d_displayDebugWindow.getValue()) return;
#endif  // SOFAOR_NEW_GUI_ENABLED
  applyFilter(d_img.getValue(), m_debugImage, true);
  if (m_debugImage.empty()) return;

#ifndef SOFAOR_NEW_GUI_ENABLED
  cv::imshow(m_win_name, m_debugImage);
#endif  // SOFAOR_NEW_GUI_ENABLED
}

void ImageFilter::activateMouseCallback()
{
  m_isMouseCallbackActive = !m_isMouseCallbackActive;
}

void ImageFilter::unregisterAllData() { m_params.clear(); }

void ImageFilter::registerData(sofa::Data<bool>* data)
{
  m_params.push_back(new ScalarSliderManager<bool>(data));
}

void ImageFilter::registerData(sofa::Data<sofa::helper::OptionsGroup>* data)
{
  m_params.push_back(
      new CustomSliderManager<sofa::helper::OptionsGroup, int>(data));
}

void ImageFilter::registerData(sofa::Data<int>* data, int min, int max,
                               int step)
{
  m_params.push_back(new ScalarSliderManager<int>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<unsigned>* data, unsigned min,
                               unsigned max, unsigned step)
{
  m_params.push_back(new ScalarSliderManager<unsigned>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<double>* data, double min, double max,
                               double step)
{
  m_params.push_back(new ScalarSliderManager<double>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<float>* data, float min, float max,
                               float step)
{
  m_params.push_back(new ScalarSliderManager<float>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2u>* data,
                               unsigned min, unsigned max, unsigned step)
{
  m_params.push_back(new VecSliderManager<2, unsigned>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3u>* data,
                               unsigned min, unsigned max, unsigned step)
{
  m_params.push_back(new VecSliderManager<3, unsigned>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4u>* data,
                               unsigned min, unsigned max, unsigned step)
{
  m_params.push_back(new VecSliderManager<4, unsigned>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2i>* data,
                               int min, int max, int step)
{
  m_params.push_back(new VecSliderManager<2, int>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3i>* data,
                               int min, int max, int step)
{
  m_params.push_back(new VecSliderManager<3, int>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4i>* data,
                               int min, int max, int step)
{
  m_params.push_back(new VecSliderManager<4, int>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2f>* data,
                               float min, float max, float step)
{
  m_params.push_back(new VecSliderManager<2, float>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3f>* data,
                               float min, float max, float step)
{
  m_params.push_back(new VecSliderManager<3, float>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4f>* data,
                               float min, float max, float step)
{
  m_params.push_back(new VecSliderManager<4, float>(data, min, max, step));
}

void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec2d>* data,
                               double min, double max, double step)
{
  m_params.push_back(new VecSliderManager<2, double>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec3d>* data,
                               double min, double max, double step)
{
  m_params.push_back(new VecSliderManager<3, double>(data, min, max, step));
}
void ImageFilter::registerData(sofa::Data<sofa::defaulttype::Vec4d>* data,
                               double min, double max, double step)
{
  m_params.push_back(new VecSliderManager<4, double>(data, min, max, step));
}

}  // namespace processor
}  // namespace sofaor
