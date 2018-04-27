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

#include "MorphologyEx.h"
#include <opencv2/imgproc.hpp>

namespace sofacv
{
namespace imgproc
{

MorphologyEx::MorphologyEx()
    : d_ksize(initData(&d_ksize, 7, "ksize", "kernel size (3 5 7 ...)")),
      d_operator(initData(&d_operator, "operator",
                          "kind of morphology operation to be performed "
                          "(OPEN, CLOSE, GRADIENT, TOPHAT, BLACKHAT")),
      d_element(initData(&d_element, "element",
                         "kernel element to be used (available shapes: RECT, "
                         "CROSS, ELLIPSE"))
{
  sofa::helper::OptionsGroup* t = d_operator.beginEdit();
  t->setNames(5, "OPEN", "CLOSE", "GRADIENT", "TOPHAT", "BLACKHAT");
  t->setSelectedItem("TOPHAT");
  d_operator.endEdit();

  t = d_element.beginEdit();
  t->setNames(3, "RECT", "CROSS", "ELLIPSE");
  t->setSelectedItem("ELLIPSE");
  d_element.endEdit();
}

void MorphologyEx::init()
{
  registerData(&d_operator);
  registerData(&d_element);
  registerData(&d_ksize, 0, 251, 1);
  ImageFilter::init();
}

void MorphologyEx::applyFilter(const cv::Mat& in,
                                                           cv::Mat& out, bool)
{
  if (in.empty()) return;

  try
  {
    int operation = int(d_operator.getValue().getSelectedId() + 2);
    cv::Mat element = cv::getStructuringElement(
        int(d_element.getValue().getSelectedId()),
        cv::Size(d_ksize.getValue() * 2 + 1, d_ksize.getValue() * 2 + 1),
        cv::Point(d_ksize.getValue(), d_ksize.getValue()));
    cv::morphologyEx(in, out, operation, element);
  }
  catch (cv::Exception& e)
  {
    std::cout << e.what() << std::endl;
    return;
  }
}

SOFA_DECL_CLASS(MorphologyEx)

int MorphologyExClass =
    sofa::core::RegisterObject(
        "OpenCV's implementation of a opencv's morphology operators")
        .add<MorphologyEx>();

}  // namespace imgproc
}  // namespace sofacv

