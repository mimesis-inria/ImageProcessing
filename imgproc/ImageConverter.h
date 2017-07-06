#ifndef SOFA_OR_PROCESSOR_IMAGECONVERTER_H
#define SOFA_OR_PROCESSOR_IMAGECONVERTER_H

#include <opencv2/imgproc.hpp>
#include "common/ImageFilter.h"

namespace sofaor
{
namespace processor
{
namespace imgproc
{
class ImageConverter : public ImageFilter
{
 public:
	SOFA_CLASS(ImageConverter, ImageFilter);

	sofa::Data<int> d_convertTo;
	sofa::Data<double> d_scaleFactor;

	ImageConverter()
			: d_convertTo(initData(&d_convertTo, "convertTo",
															"opencv type value")),
				d_scaleFactor(initData(&d_scaleFactor, "scaleFactor", "scale Factor"))
  {
  }

  void init()
  {
    ImageFilter::init();
  }

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool)
  {
    if (in.empty()) return;
		in.convertTo(out, d_convertTo.getValue(), d_scaleFactor.getValue());
  }
};

SOFA_DECL_CLASS(ImageConverter)

int ImageConverterClass =
		sofa::core::RegisterObject("Image type converter")
				.add<ImageConverter>();

}  // namespace imgproc
}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_IMAGECONVERTER_H
