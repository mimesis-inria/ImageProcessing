#include "TemplateMatcher.h"

namespace sofacv
{
namespace imgproc
{
TemplateMatcher::TemplateMatcher()
    : d_template(initData(&d_template, "template_img",
                          "template image to search for in the input img.")),
      d_method(initData(&d_method, "method",
                        "comparison method to use for matching."))
{
  sofa::helper::OptionsGroup *t = d_method.beginEdit();
  t->setNames(6, "SQDIFF", "SQDIFF_NORMED", "CCORR", "CCORR_NORMED", "CCOEFF",
              "CCOEFF_NORMED");
  t->setSelectedItem("SQDIFF");
  d_method.endEdit();
}

void TemplateMatcher::init()
{
  addInput(&d_template);
  registerData(&d_method);
  ImageFilter::init();
}

void TemplateMatcher::applyFilter(const cv::Mat &in, cv::Mat &out, bool)
{
  if (in.empty()) return;

  if (in.type() != CV_8UC1)
    msg_error("TemplateMatcher::applyFilter()")
        << "TemplateMatcher::img must be grayscale";
  if (d_template.getValue().type() != CV_8UC1)
    msg_error("TemplateMatcher::applyFilter()")
        << "TemplateMatcher::template_img must be grayscale";

  out = in.clone();

  cv::matchTemplate(in, d_template.getValue(), out,
                    int(d_method.getValue().getSelectedId()));

  /// Localizing the best match with minMaxLoc
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;
  cv::Point matchLoc;
  cv::minMaxLoc(out, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

  cv::normalize(out, out, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
  cv::minMaxLoc(out, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

  cv::Mat img_display;
  in.copyTo(img_display);

  /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all
  /// the other methods, the higher the better
  if (d_method.getValue().getSelectedItem() == "TM_SQDIFF" ||
      d_method.getValue().getSelectedItem() == "TM_SQDIFF_NORMED")
  {
    matchLoc = minLoc;
  }
  else
  {
    matchLoc = maxLoc;
  }

  cv::Mat templ = d_template.getValue().clone();

  /// Show me what you got
  cv::rectangle(img_display, matchLoc,
                cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
                cv::Scalar::all(0), 2, 8, 0);
  cv::rectangle(out, matchLoc,
                cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
                cv::Scalar::all(0), 2, 8, 0);
}

SOFA_DECL_CLASS(TemplateMatcher)

int TemplateMatcherClass = sofa::core::RegisterObject(
                               "the 6 Template matching algorithms from "
                               "OpenCV's cv::matchTemplate() method")
                               .add<TemplateMatcher>();

}  // namespace imgproc
}  // namespace sofacv
