#ifndef SOFACV_IMGPROC_CONVERTTO_H
#define SOFACV_IMGPROC_CONVERTTO_H

#include <SofaCV/SofaCV.h>
#include "ImageProcessingPlugin.h"

#define SHOWVAL(v) #v

namespace sofacv
{
namespace imgproc
{
/**
 * @brief The ConvertTo class
 *
 * Converts a cvMat to another with differento data type and optional scaling of
 * the values. (equivalent of the cv::Mat::convertTo() method) This is used to,
 * for instance, convert a CV_32F to a CV_8U by scaling every values from
 * 0 < val < 1 to 0 < val < 256 for instance.
 */
template <class T>
class SOFA_IMAGEPROCESSING_API ConvertTo : public ImageFilter
{
  T internal_type;

 public:
  SOFA_CLASS(ConvertTo, ImageFilter);

  sofa::Data<double> d_alpha;
  sofa::Data<double> d_beta;

  ConvertTo();

  void init() override;

  void applyFilter(const cv::Mat& in, cv::Mat& out, bool) override;

  int getCVType(T type);

  virtual std::string getTemplateName() const { return templateName(this); }
  static std::string templateName(const ConvertTo<T>* = NULL);
};

}  // namespace imgproc
}  // namespace sofacv
#endif  // SOFACV_IMGPROC_CONVERTTO_H
