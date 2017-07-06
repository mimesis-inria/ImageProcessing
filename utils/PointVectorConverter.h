#ifndef SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
#define SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H

#include "camera/common/CameraSettings.h"
#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvKeypoint.h>

namespace sofaor
{
namespace processor
{
namespace utils
{
template <class SrcType, class DstType>
class PointVectorConverter : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<
			PointVectorConverter, cam::CameraSettings,
			sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	SOFA_CLASS(SOFA_TEMPLATE2(PointVectorConverter, SrcType, DstType),
						 common::ImplicitDataEngine);

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

	~PointVectorConverter() {}
	void init()
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

	void update();

	virtual std::string getTemplateName() const { return templateName(this); }
	static std::string templateName(
			const PointVectorConverter<SrcType, DstType>* = NULL);

	// INPUTS
	sofa::Data<sofa::helper::vector<SrcType> > d_src;
	// OUTPUTS
	sofa::Data<sofa::helper::vector<DstType> > d_dst;

	CamSettings l_cam;
	sofa::Data<double> d_depth;
};

}  // namespace utils
}  // namespace processor
}  // namespace sofaor

#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
