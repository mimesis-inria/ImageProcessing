#ifndef SOFA_OR_PROCESSOR_MORPHOLOGYEX_H
#define SOFA_OR_PROCESSOR_MORPHOLOGYEX_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class MorphologyEx : public ImageFilter
{
 public:
  SOFA_CLASS(MorphologyEx, ImageFilter);

	sofa::Data<int> d_ksize;
	sofa::Data<sofa::helper::OptionsGroup> d_operator;
	sofa::Data<sofa::helper::OptionsGroup> d_element;

  MorphologyEx()
      : d_ksize(initData(&d_ksize, 5, "ksize", "kernel size (3 5 7 ...)")),
        d_operator(initData(&d_operator, "operator",
                            "kind of morphology operation to be performed "
                            "(OPEN, CLOSE, GRADIENT, TOPHAT, BLACKHAT")),
        d_element(initData(&d_element, "element",
                           "kernel element to be used (available shapes: RECT, "
                           "CROSS, ELLIPSE"))
  {
		sofa::helper::OptionsGroup* t = d_operator.beginEdit();
    t->setNames(5, "OPEN", "CLOSE", "GRADIENT", "TOPHAT", "BLACKHAT");
    t->setSelectedItem("OPEN");
    d_operator.endEdit();

    t = d_element.beginEdit();
    t->setNames(3, "RECT", "CROSS", "ELLIPSE");
    t->setSelectedItem("RECT");
    d_element.endEdit();
  }

  void init()
  {
    registerData(&d_operator);
    registerData(&d_element);
    registerData(&d_ksize, 0, 251, 1);
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;

    try
    {
      int operation = d_operator.getValue().getSelectedId() + 2;
			cv::Mat element = cv::getStructuringElement(
					d_element.getValue().getSelectedId(),
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
};

SOFA_DECL_CLASS(MorphologyEx)

int MorphologyExClass =
		sofa::core::RegisterObject(
        "OpenCV's implementation of a opencv's morphology operators")
        .add<MorphologyEx>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_MORPHOLOGYEX_H
