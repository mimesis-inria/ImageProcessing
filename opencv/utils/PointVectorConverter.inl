//#include "PointVectorConverter.h"

//#include <SofaORCommon/cvKeypoint.h>

//namespace sofa
//{
//namespace OR
//{
//namespace processor
//{

//template<>
//void PointVectorConverter<cvKeypoint2Vec2i>::update()
//{
//    helper::vector<defaulttype::Vec2i>& dst = *(d_dst.beginWriteOnly());
//    dst.clear();
//    const helper::vector<common::cvKeypoint>& src = d_src.getValue();
//    for (auto kp : src)
//        dst.push_back(defaulttype::Vec2i(kp.pt.x, kp.pt.y));
//}

//template<>
//void PointVectorConverter<Vec2i2cvKeypoint>::update()
//{
//    helper::vector<common::cvKeypoint>& dst = *(d_dst.beginWriteOnly());
//    dst.clear();
//    const helper::vector<defaulttype::Vec2i>& src = d_src.getValue();
//    for (auto pt : src)
//        dst.push_back(common::cvKeypoint(cv::Point2f(pt.x(), pt.y()), 0));
//}


//} // namespace processor
//} // namespace OR
//} // namespace sofa
