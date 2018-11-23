#ifndef SOFACV_UTILS_POINTVECTORCONVERTER_H
#define SOFACV_UTILS_POINTVECTORCONVERTER_H

#include "ImageProcessingPlugin.h"
#include "camera/common/CameraSettings.h"

#include <SofaCV/SofaCV.h>

namespace sofacv
{
namespace utils
{
template <class SrcType, class DstType>
class SOFA_IMAGEPROCESSING_API PointVectorConverter : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      PointVectorConverter, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;

 public:
  SOFA_CLASS(SOFA_TEMPLATE2(PointVectorConverter, SrcType, DstType),
             ImplicitDataEngine);

  PointVectorConverter()
      : l_cam(initLink("cam",
                       "link to CameraSettings component to reproject 2D "
                       "points in 3D n vice versa")),
        d_src(initData(&d_src, "points", "input vector to convert")),
        d_dst(initData(&d_dst, "points_out", "converted output vector")),
        d_depth(initData(
            &d_depth, "depth",
            "depth at which you want to see your image's points projected"))
  {
  }

  ~PointVectorConverter() override {}
  void init() override
  {
    addInput(&d_src);
    addInput(&d_depth);

    addOutput(&d_dst);

    if (!l_cam.get())
      msg_warning(getName() + "::init()") << "Warning: No camera link set. "
                                             "Please use attribute 'cam' "
                                             "to define one";

    update();
  }

  virtual void doUpdate() override;

  virtual std::string getTemplateName() const { return templateName(this); }
  static std::string templateName(
      const PointVectorConverter<SrcType, DstType>* = nullptr);

  CamSettings l_cam;

  // INPUTS
  sofa::Data<sofa::helper::vector<SrcType> > d_src;
  // OUTPUTS
  sofa::Data<sofa::helper::vector<DstType> > d_dst;

  sofa::Data<double> d_depth;
};

}  // namespace utils
}  // namespace sofacv

#endif  // SOFACV_UTILS_POINTVECTORCONVERTER_H
