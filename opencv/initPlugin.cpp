#include "initPlugin.h"
#include <sofa/helper/system/config.h>

namespace sofa
{
namespace OR
{
namespace processor
{
// Here are just several convenient functions to help user to know what contains
// the plugin

extern "C" {
SOFA_SOFAOROPENCVPLUGIN_API void initExternalModule();
SOFA_SOFAOROPENCVPLUGIN_API const char* getModuleName();
SOFA_SOFAOROPENCVPLUGIN_API const char* getModuleVersion();
SOFA_SOFAOROPENCVPLUGIN_API const char* getModuleLicense();
SOFA_SOFAOROPENCVPLUGIN_API const char* getModuleDescription();
SOFA_SOFAOROPENCVPLUGIN_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
  }
}

const char* getModuleName() { return "SofaOROpenCV"; }
const char* getModuleVersion() { return "0.1"; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription()
{
  return "ProcessOR's OpenCV module to perform computer vision tasks such as "
         "2D mono/stereo image processing, point cloud processing etc.";
}

const char* getModuleComponentList()
{
  return "ImageExporter;FeatureDetector;FeatureDescriptor;FeatureMatcher;MatchingConstraints;"
         "FeatureTriangulator;OpticalFlow;CannyFilter;SobelFilter;"
         "ImageRectifier;CalibLoader;Segmenter2D;PointPicker2D;SimpleThreshold";
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa

////////// BEGIN CLASS LIST //////////

SOFA_LINK_CLASS(ImageExporter);
SOFA_LINK_CLASS(FeatureDetector);
SOFA_LINK_CLASS(DescriptorMatcher);
SOFA_LINK_CLASS(MatchingConstraints);
SOFA_LINK_CLASS(FeatureTriangulator);
SOFA_LINK_CLASS(OpticalFlow);
SOFA_LINK_CLASS(CannyFilter);
SOFA_LINK_CLASS(SobelFilter);
SOFA_LINK_CLASS(ImageRectifier);
SOFA_LINK_CLASS(CalibLoader);
SOFA_LINK_CLASS(Segmenter2D);
SOFA_LINK_CLASS(PointPicker2D);
SOFA_LINK_CLASS(SimpleThreshold);
