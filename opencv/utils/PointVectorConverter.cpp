#include "PointVectorConverter.inl"

#include <SofaORCommon/cvKeypoint.h>

#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(PointVectorConverter)

int PointVectorConverterClass =
    core::RegisterObject(
        "Converts vector of cvKeyPoints to sofa vectors, and vice versa")
        .add<PointVectorConverter<cvKeypoint2Sofa> >(true)
        .add<PointVectorConverter<Sofa2cvKeypoint> >();

}  // namespace processor
}  // namespace OR
}  // namespace sofa
