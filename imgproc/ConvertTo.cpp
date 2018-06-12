#include "ConvertTo.inl"

#include <sofa/core/ObjectFactory.h>

namespace sofacv
{
namespace imgproc
{
SOFA_DECL_CLASS(ConvertTo)

int ConvertToClass = sofa::core::RegisterObject(
                         "Converts OpenCV Matrices pixel depth, and optionally "
                         "scales / crop the histogram using alpha & beta. Number of channels stay the same")
                         .add<ConvertTo<char> >()
                         .add<ConvertTo<unsigned char> >()
                         .add<ConvertTo<short> >()
                         .add<ConvertTo<unsigned short> >()
                         .add<ConvertTo<int> >()
                         .add<ConvertTo<float> >()
                         .add<ConvertTo<double> >();


}  // namespace imgproc
}  // namespace sofacv
