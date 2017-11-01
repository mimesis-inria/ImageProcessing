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

#ifndef SOFA_OR_PROCESSOR_TEMPLATEMATCHER_H
#define SOFA_OR_PROCESSOR_TEMPLATEMATCHER_H

#include "common/ImageFilter.h"

#include <sofa/helper/OptionsGroup.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class TemplateMatcher : public ImageFilter
{
 public:
  SOFA_CLASS(TemplateMatcher, ImageFilter);

  sofa::Data<common::cvMat> d_template;
  sofa::Data<sofa::helper::OptionsGroup> d_method;

  TemplateMatcher()
      : d_template(initData(&d_template, "template_img",
                            "template image to search for in the input img.")),
        d_method(initData(&d_method, "method",
                          "comparison method to use for matching."))
  {
    sofa::helper::OptionsGroup* t = d_method.beginEdit();
    t->setNames(6, "SQDIFF", "SQDIFF_NORMED", "CCORR", "CCORR_NORMED", "CCOEFF",
                "CCOEFF_NORMED");
    t->setSelectedItem("SQDIFF");
    d_method.endEdit();
  }

  void init()
  {
    addInput(&d_template);
    registerData(&d_method);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;

    if (in.type() != CV_8UC1)
      msg_error("TemplateMatcher::applyFilter()")
          << "TemplateMatcher::img must be grayscale";
    if (d_template.getValue().type() != CV_8UC1)
      msg_error("TemplateMatcher::applyFilter()")
          << "TemplateMatcher::template_img must be grayscale";

    out = in.clone();
    cv::imwrite("in.png", in);
    cv::imwrite("template.png", d_template.getValue());

    cv::matchTemplate(in, d_template.getValue(), out,
                      d_method.getValue().getSelectedId());

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;
    cv::minMaxLoc( out, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    std::cout << " min val max val " << minVal << " " << maxVal << std::endl;

    cv::normalize(out,out,0,1,cv::NORM_MINMAX, -1, cv::Mat() );
    cv::minMaxLoc( out, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    cv::Mat img_display;
    in.copyTo( img_display );

    /*char* image_window = "Source Image";
    char* result_window = "Result window";

    cv::namedWindow( image_window, CV_WINDOW_AUTOSIZE );
    cv::namedWindow( result_window, CV_WINDOW_AUTOSIZE );*/

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    //if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
     // { matchLoc = minLoc; }
    //else
      { matchLoc = maxLoc; }

    cv::Mat templ = d_template.getValue().clone();

    /// Show me what you got
    cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
    cv::rectangle( out, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

    //cv::imshow( image_window, img_display );
    //cv::imshow( result_window, out );
    cv::imwrite("img_display.png", img_display);

  }
};

SOFA_DECL_CLASS(TemplateMatcher)

int TemplateMatcherClass = sofa::core::RegisterObject(
                               "the 6 Template matching algorithms from "
                               "OpenCV's cv::matchTemplate() method")
                               .add<TemplateMatcher>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_TEMPLATEMATCHER_H
