#ifndef SOFA_OR_PROCESSOR_TEMPLATEMATCHER_H
#define SOFA_OR_PROCESSOR_TEMPLATEMATCHER_H

#include "common/ImageFilter.h"

#include <sofa/helper/OptionsGroup.h>

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
		registerData(&d_method);
		ImageFilter::init();
	}

	void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
	{
		if (in.empty()) return;
		cv::Mat img_gray;
		if (in.type() == CV_8UC4)
			cv::cvtColor(in, img_gray, CV_BGRA2GRAY);
		else if (in.type() == CV_8UC3)
			cv::cvtColor(in, img_gray, CV_BGR2GRAY);
		else
			img_gray = in.clone();

		cv::Mat templ_gray;
		if (d_template.getValue().type() == CV_8UC4)
			cv::cvtColor(d_template.getValue(), templ_gray, CV_BGRA2GRAY);
		else if (d_template.getValue().type() == CV_8UC3)
			cv::cvtColor(d_template.getValue(), templ_gray, CV_BGR2GRAY);
		else
			templ_gray = d_template.getValue().clone();

		cv::matchTemplate(img_gray, templ_gray, out,
											d_method.getValue().getSelectedId());
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
