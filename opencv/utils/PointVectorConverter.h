//#ifndef SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
//#define SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H

//#include "initPlugin.h"

//#include <SofaORCommon/ImplicitDataEngine.h>
//#include <SofaORCommon/DataTypes.h>

//namespace sofa
//{
//namespace OR
//{
//namespace processor
//{

//template <class ConversionType>
//class PointVectorConverter : public common::ImplicitDataEngine
//{
// public:
//	SOFA_CLASS(SOFA_TEMPLATE(PointVectorConverter, ConversionType),
//             common::ImplicitDataEngine);

//  PointVectorConverter()
//      : d_src(initData(&d_src, "points", "input vector to convert")),
//        d_dst(initData(&d_dst, "points_out", "converted output vector"))
//  {
//  }

//  ~PointVectorConverter() {}
//  void init()
//  {
//    addInput(&d_src);
//    addOutput(&d_dst);
//  }

//  void update();

//  // INPUTS
//	Data<helper::vector<typename ConversionType::SrcType> > d_src;
//  // OUTPUTS
//	Data<helper::vector<typename ConversionType::DstType> > d_dst;
//};

//}  // namespace processor
//}  // namespace OR
//}  // namespace sofa

//#endif  // SOFA_OR_PROCESSOR_POINTVECTORCONVERTER_H
