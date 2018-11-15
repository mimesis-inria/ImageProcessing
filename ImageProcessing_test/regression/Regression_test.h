#ifndef SOFACV_REGRESSION_TEST_H
#define SOFACV_REGRESSION_TEST_H

#include <SofaSimulationGraph/DAGSimulation.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/simulation/Visitor.h>

#include <SofaComponentAdvanced/initComponentAdvanced.h>
#include <SofaComponentBase/initComponentBase.h>
#include <SofaComponentCommon/initComponentCommon.h>
#include <SofaComponentGeneral/initComponentGeneral.h>
#include <SofaComponentMisc/initComponentMisc.h>

#include <../extlibs/json/json.h>
#include <SofaTest/Sofa_test.h>

#include <openssl/md5.h>
#include <zlib.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

// Code taken from http://panthema.net/2007/0328-ZLibString.html
/** Compress a STL string using zlib with given compression level and return
  * the binary data. */
std::string compress_string(const std::string& str,
                            int compressionlevel = Z_BEST_COMPRESSION)
{
  z_stream zs;  // z_stream is zlib's control structure
  memset(&zs, 0, sizeof(zs));

  if (deflateInit(&zs, compressionlevel) != Z_OK)
    throw(std::runtime_error("deflateInit failed while compressing."));

  zs.next_in = (Bytef*)str.data();
  zs.avail_in = str.size();  // set the z_stream's input

  int ret;
  char outbuffer[32768];
  std::string outstring;

  // retrieve the compressed bytes blockwise
  do
  {
    zs.next_out = reinterpret_cast<Bytef*>(outbuffer);
    zs.avail_out = sizeof(outbuffer);

    ret = deflate(&zs, Z_FINISH);

    if (outstring.size() < zs.total_out)
    {
      // append the block to the output string
      outstring.append(outbuffer, zs.total_out - outstring.size());
    }
  } while (ret == Z_OK);

  deflateEnd(&zs);

  if (ret != Z_STREAM_END)
  {  // an error occurred that was not EOF
    std::ostringstream oss;
    oss << "Exception during zlib compression: (" << ret << ") " << zs.msg;
    throw(std::runtime_error(oss.str()));
  }

  return outstring;
}

/** Decompress an STL string using zlib and return the original data. */
std::string decompress_string(const std::string& str)
{
  z_stream zs;  // z_stream is zlib's control structure
  memset(&zs, 0, sizeof(zs));

  if (inflateInit(&zs) != Z_OK)
    throw(std::runtime_error("inflateInit failed while decompressing."));

  zs.next_in = (Bytef*)str.data();
  zs.avail_in = str.size();

  int ret;
  char outbuffer[32768];
  std::string outstring;

  // get the decompressed bytes blockwise using repeated calls to inflate
  do
  {
    zs.next_out = reinterpret_cast<Bytef*>(outbuffer);
    zs.avail_out = sizeof(outbuffer);

    ret = inflate(&zs, 0);

    if (outstring.size() < zs.total_out)
    {
      outstring.append(outbuffer, zs.total_out - outstring.size());
    }

  } while (ret == Z_OK);

  inflateEnd(&zs);

  if (ret != Z_STREAM_END)
  {  // an error occurred that was not EOF
    std::ostringstream oss;
    oss << "Exception during zlib decompression: (" << ret << ") " << zs.msg;
    throw(std::runtime_error(oss.str()));
  }

  return outstring;
}

namespace sofa
{
/// To Perform a Regression Test on scenes, based on their Component's Data
///
/// This Regression test API provides the same mechanism as Regression_test,
/// except that the comparison predicate is based on the component's data fields
/// instead of the MechanicalObjects' states
///
/// @author Bruno Marques
/// @date 2018
///

using json = sofa::helper::json;

/// Checks a reference json file agains the actual values for regression
class CompareDataVisitor : public sofa::simulation::Visitor
{
 public:
  CompareDataVisitor(const core::ExecParams* params, const std::string& n,
                     const std::string& ref)
      : sofa::simulation::Visitor(params)
  {
    std::ifstream i(ref.c_str());
    std::string str((std::istreambuf_iterator<char>(i)),
                    std::istreambuf_iterator<char>());
    m_reference = json::parse(::decompress_string(str).c_str());
    std::cout << m_reference << std::endl;
    m_sceneName = n;
  }

  void addError(sofa::core::objectmodel::BaseObject* obj,
                sofa::core::objectmodel::BaseData* p, std::string val,
                std::string ref)
  {
    m_ss << "- Data differs for " << obj->getName() << "." << p->getName()
         << std::endl;
    if (p->getValueTypeInfo()->name() != "cvMat")
      m_ss << "    Current value: "
           << val << "\n    Value of reference: "
           << ref << std::endl;
  }

  simulation::Visitor::Result processNodeTopDown(simulation::Node* node)
  {
    std::vector<sofa::core::objectmodel::BaseObject*> objs;
    node->getContext()->getObjects(objs);
    for (const auto& obj : objs)
    {
      for (const auto& p : obj->getDataFields())
      {
        // if data field doesn't exist in reference:
        if (m_reference[obj->getName()].empty() ||
            m_reference[obj->getName()][p->getName()].empty())
        {
          m_ss << "- No Data field in reference for " + obj->getName() + "." +
                      p->getName()
               << std::endl;
          continue;
        }
        std::string refValue = m_reference[obj->getName()][p->getName()];
        std::string s = p->getValueString();
        if (p->getValueTypeInfo()->name() == "cvMat" || s.size() > 256)
        {
          unsigned char md5sum[MD5_DIGEST_LENGTH];
          MD5((unsigned char*)s.c_str(), s.size(), md5sum);
          std::string hash = "";
          for (int i = 0; i < MD5_DIGEST_LENGTH; ++i)
            hash += std::to_string(md5sum[i]);
          if (hash != refValue)
          {
            addError(obj, p, refValue, hash);
          }
        }
        else if (refValue != p->getValueString())
        {
          addError(obj, p, refValue, p->getValueString());
        }
      }
    }
    return RESULT_CONTINUE;
  }

  void setSceneName(std::string& n) { m_sceneName = n; }
  const char* getClassName() const { return "CompareDataVisitor"; }
  const std::string getErrorList() { return m_ss.str(); }

 protected:
  std::stringstream m_ss;
  std::string m_sceneName;
  json m_reference;
};

/// Generates a reference json file for the scene
class DataExporterVisitor : public sofa::simulation::Visitor
{
 public:
  DataExporterVisitor(const core::ExecParams* params, const std::string& n)
      : sofa::simulation::Visitor(params)
  {
    m_sceneName = n;
  }

  simulation::Visitor::Result processNodeTopDown(simulation::Node* node)
  {
    std::vector<sofa::core::objectmodel::BaseObject*> objs;
    node->getContext()->getObjects(objs);
    for (const auto& obj : objs)
    {
      for (const auto& p : obj->getDataFields())
      {
        std::string s = p->getValueString();
        unsigned char md5sum[MD5_DIGEST_LENGTH];
        if (p->getValueTypeInfo()->name() == "cvMat" || s.size() > 256)
        {
          MD5((unsigned char*)s.c_str(), s.size(), md5sum);
          std::string hash = "";
          for (int i = 0; i < MD5_DIGEST_LENGTH; ++i)
            hash += std::to_string(md5sum[i]);

          m_dump[obj->getName()][p->getName()] = hash;
        }
        else
          m_dump[obj->getName()][p->getName()] = p->getValueString();
      }
    }
    return RESULT_CONTINUE;
  }

  void setSceneName(std::string& n) { m_sceneName = n; }
  void saveJson(const std::string& ref)
  {
    std::ofstream o(ref.c_str());
    if (o.is_open())
    {
      std::stringstream ss;
      ss << m_dump;
      o << ::compress_string(ss.str());
      o.close();
    }
  }
  const char* getClassName() const { return "DataExporterVisitor"; }

 protected:
  std::string m_sceneName;
  json m_dump;
};

class DataRegression_test : public testing::Test
{
 protected:
  void runRegressionScene(std::string& reference, const std::string& scene,
                          unsigned int steps)
  {
    msg_info("Regression_test") << "  Testing " << scene;

    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

    simulation::Simulation* simu = simulation::getSimulation();

    // Load the scene
    sofa::simulation::Node::SPtr root = simu->load(scene.c_str());

    simu->init(root.get());

    bool initializing = false;

    sofa::simulation::Visitor* visitor = nullptr;

    // if a ref file already exists
    if (helper::system::FileSystem::exists(reference) &&
        !helper::system::FileSystem::isDirectory(reference))
    {
      visitor = new CompareDataVisitor(
          sofa::core::ExecParams::defaultInstance(), reference, reference);
    }
    else  // create reference
    {
      initializing = true;
      msg_warning("Regression_test")
          << "Creating non-existing reference: " << reference;
      visitor = new DataExporterVisitor(
          sofa::core::ExecParams::defaultInstance(), reference);
    }

    for (unsigned int i = 0; i < steps; ++i)
    {
      simu->animate(root.get(), root->getDt());
    }

    visitor->execute(root.get());

    if (initializing)
    {
      dynamic_cast<DataExporterVisitor*>(visitor)->saveJson(reference);
    }
    else
    {
      std::string errors =
          dynamic_cast<CompareDataVisitor*>(visitor)->getErrorList();
      if (errors != "") ADD_FAILURE() << scene << ":\n" << errors;
    }

    if (visitor)
      delete visitor;
    visitor = nullptr;
    // Clear and prepare for next scene
    try {
        simu->unload(root.get());
    } catch (std::exception& e) {
        msg_error("REGRESSION") << "Could not unload simulation properly:\n" << e.what();
    }
    root.reset();
  }

  /// - scenesDir is the directory containing the scenes
  /// - testDir is the hidden folder containing the regression archives
  void runRegressionList(const std::string& scenesDir,
                         const std::string& testDir)
  {
    // lire plugin_test/regression_scene_list -> (file,nb time steps,epsilon)
    // pour toutes les scenes

    const std::string regression_scene_list = testDir + "list.txt";

    if (helper::system::FileSystem::exists(regression_scene_list) &&
        !helper::system::FileSystem::isDirectory(regression_scene_list))
    {
      msg_info("Regression_test") << "Parsing " << regression_scene_list;

      // parser le fichier -> (file,nb time steps,epsilon)
      std::ifstream iniFileStream(regression_scene_list.c_str());
      while (!iniFileStream.eof())
      {
        std::string line;
        std::string scene;
        unsigned int steps;
        double epsilon;

        getline(iniFileStream, line);
        if (line[0] == '#') continue;
        std::istringstream lineStream(line);
        lineStream >> scene;
        lineStream >> steps;
        lineStream >> epsilon;

        if (scene == "") break;

        scene = scenesDir + scene;
        std::string reference = testDir + getFileName(scene) + ".ref";

#ifdef WIN32
        // Minimize absolute scene path to avoid MAX_PATH problem
        if (scene.length() > MAX_PATH)
        {
          ADD_FAILURE() << scene << ": path is longer than " << MAX_PATH;
          continue;
        }
        char buffer[MAX_PATH];
        GetFullPathNameA(scene.c_str(), MAX_PATH, buffer, nullptr);
        scene = std::string(buffer);
        std::replace(scene.begin(), scene.end(), '\\', '/');
#endif  // WIN32

        std::cout << "running regression scene: " << reference << " " << scene
                  << " " << steps << std::endl;

        runRegressionScene(reference, scene, steps);
      }
    }
    sofa::helper::system::DataRepository.displayPaths();
  }

  std::string getFileName(const std::string& s)
  {
    char sep = '/';

    size_t i = s.rfind(sep, s.length());
    if (i != std::string::npos)
    {
      return (s.substr(i + 1, s.length() - i));
    }

    return s;
  }

  void testTestPath(const std::string& scenesDirectory)
  {
    // pour tous plugins/projets
    std::vector<std::string> dir;
    helper::system::FileSystem::listDirectory(scenesDirectory, dir);

    if (helper::system::FileSystem::isDirectory(scenesDirectory))
    {
      // regression folder is hidden to avoid polluting the repo
      const std::string testDir = scenesDirectory + ".regression/";

      if (helper::system::FileSystem::exists(testDir) &&
          helper::system::FileSystem::isDirectory(testDir))
      {
        runRegressionList(scenesDirectory, testDir);
      }
    }
  }

  // Create the context for the scene
  virtual void SetUp()
  {
    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

    sofa::simulation::setSimulation(
        new sofa::simulation::graph::DAGSimulation());

    static const std::string examples_Dir =
        std::string(ImageProcessing_REGRESSION_DIR);
    if (helper::system::FileSystem::exists(examples_Dir))
      testTestPath(examples_Dir);
  }
};

}  // namespace sofa

#endif  // SOFACV_REGRESSION_TEST_H
