#include <sofa/helper/system/config.h>
#include "initplugin.h"

namespace sofa
{
namespace OR
{
namespace processor
{
// Here are just several convenient functions to help user to know what contains
// the plugin

extern "C" {
SOFA_PROCESSORPLUGIN_API void initExternalModule();
SOFA_PROCESSORPLUGIN_API const char* getModuleName();
SOFA_PROCESSORPLUGIN_API const char* getModuleVersion();
SOFA_PROCESSORPLUGIN_API const char* getModuleLicense();
SOFA_PROCESSORPLUGIN_API const char* getModuleDescription();
SOFA_PROCESSORPLUGIN_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
  }
}

const char* getModuleName() { return "ProcessOR"; }
const char* getModuleVersion() { return "0.1"; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription()
{
  return "ProcessOR module to perform computer vision tasks such as 2D mono/stereo image processing, point cloud processing etc.";
}

const char* getModuleComponentList()
{
  return "FeatureDetector;FeatureDescriptor;FeatureMatcher;MatchingConstraints;FeatureTriangulator;FeatureRectifier;CannyFilter;OpticalFlow"
#ifdef SOFAOR_ENABLE_PCL
         ";PCSmootherMLS;PCDownsampler"
#endif  // SOFAOR_ENABLE_PCL
      ;
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa

////////// BEGIN CLASS LIST //////////

SOFA_LINK_CLASS(FeatureDetector);
SOFA_LINK_CLASS(DescriptorMatcher);
SOFA_LINK_CLASS(MatchingConstraints);
SOFA_LINK_CLASS(CannyFilter);
SOFA_LINK_CLASS(FeatureRectifier);
SOFA_LINK_CLASS(FeatureTriangulator);
SOFA_LINK_CLASS(OpticalFlow);

#ifdef SOFAOR_ENABLE_PCL
SOFA_LINK_CLASS(PCSmootherMLS);
SOFA_LINK_CLASS(PCDownsampler);
#endif  // SOFAOR_ENABLE_PCL
