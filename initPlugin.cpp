#include <sofa/helper/system/config.h>
#include "initPlugin.h"

namespace sofaor
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
  return "ProcessOR's Base module";
}

const char* getModuleComponentList()
{
  return "";
}

}  // namespace processor
}  // namespace sofaor
////////// BEGIN CLASS LIST //////////

