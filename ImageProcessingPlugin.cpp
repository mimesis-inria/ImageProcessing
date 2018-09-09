#include "ImageProcessingPlugin.h"
#include <sofa/helper/system/config.h>

namespace sofacv
{
/**
 * \brief Data processing library
 *  provides 2D / volumic image, point cloud and sensors data processing
 * algorithms to extract simulation-relevant data.
 */

namespace features
{
}  // namespace features

/**
 * \brief Image processing filters
 *
 * implements ImageFilter, and provides image filtering components such as
 * Sobel, Canny etc.
 */
namespace imgproc
{
}  // namespace imgproc

// Here are just several convenient functions to help user to know what contains
// the plugin

extern "C" {
SOFA_IMAGEPROCESSING_API void initExternalModule();
SOFA_IMAGEPROCESSING_API const char* getModuleName();
SOFA_IMAGEPROCESSING_API const char* getModuleVersion();
SOFA_IMAGEPROCESSING_API const char* getModuleLicense();
SOFA_IMAGEPROCESSING_API const char* getModuleDescription();
SOFA_IMAGEPROCESSING_API const char* getModuleComponentList();
}

void initExternalModule()
{
  static bool first = true;
  if (first)
  {
    first = false;
  }
}

const char* getModuleName() { return "ImageProcessing"; }
const char* getModuleVersion() { return ImageProcessing_VERSION; }
const char* getModuleLicense() { return ""; }
const char* getModuleDescription()
{
  return "SofaCV's Image processing module";
}

const char* getModuleComponentList() { return ""; }

}  // namespace sofacv
////////// BEGIN CLASS LIST //////////
