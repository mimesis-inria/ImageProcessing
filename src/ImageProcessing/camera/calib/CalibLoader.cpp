#include "CalibLoader.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <cstring>
#include <fstream>
#include <string>

#include <opencv2/core/persistence.hpp>

namespace sofacv
{
namespace cam
{
namespace calib
{
SOFA_DECL_CLASS(CalibLoader)

int CalibLoaderClass = sofa::core::RegisterObject(
                           "Mono / stereo Camera calibration settings loader")
                           .add<CalibLoader>();

CalibLoader::CalibLoader()
    : l_sCam(initLink("stereoCam",
                      "Link to the StereoSettings component holding the stereo "
                      "settings (fundamental matrix etc)")),
      l_cam1(initLink("cam",
                      "link to CameraSettings component containing and "
                      "maintaining the camera's parameters")),
      l_cam2(initLink("cam2",
                      "link to a second CameraSettings component in case of "
                      "stereo calibration data")),
      d_calibFolder(initData(&d_calibFolder, "calibDir",
                             "directory in which calibrations are stored")),
      d_calibNames(initData(&d_calibNames, "calibName",
                            "name of the calib settings currently used")),
      m_isStereo(true)
{
  f_listening.setValue(true);
  m_isInitialized = false;
}

CalibLoader::~CalibLoader() {}
bool CalibLoader::canLoad(const std::string& calibfile) const
{
  if (calibfile == "")
  {
    msg_error("CalibLoader::canLoad()")
        << "Error: CalibLoader was given an empty calibName";
    return false;
  }

  // -- Check if file exist:
  const char* filename = calibfile.c_str();
  std::string sfilename(filename);

  sofa::helper::system::FileRepository fr;
  if (!fr.findFile(sfilename))
  {
    msg_error("CalibLoader::canLoad()")
        << "Error: File '" << calibfile << "' not found. ";
    return false;
  }

  std::ifstream f(filename);

  // -- Check if file is readable:
  if (!f.good())
  {
    msg_error("CalibLoader::canLoad()")
        << "Error: Cannot read file '" << calibfile << "'.";
    return false;
  }
  f.close();
  return true;
}

void CalibLoader::setCurrentCalib(CalibData& d)
{
  if (!m_isInitialized) return;
  d_K1.setValue(d.K1);
  d_K2.setValue(d.K2);
  d_R1.setValue(d.R1);
  d_R2.setValue(d.R2);
  d_T1.setValue(d.T1);
  d_T2.setValue(d.T2);
  d_delta1.setValue(d.delta1);
  d_delta2.setValue(d.delta2);
  d_imSize1.setValue(d.imSize1);
  d_imSize2.setValue(d.imSize2);
  d_error1.setValue(d.error1);
  d_error2.setValue(d.error2);
  d_Rs.setValue(d.Rs);
  d_Ts.setValue(d.Ts);
  d_F.setValue(d.F);
  d_F.setValue(d.E);
  d_totalError.setValue(d.totalError);

  msg_info(getName()) << "loading camera data from "
                      << d_calibNames.getValue().getSelectedItem();
  if (m_isStereo)
  {
    l_sCam->setFundamentalMatrix(d.F);
    l_sCam->setEssentialMatrix(d.E);
    l_sCam->getCamera2().setImageSize(d.imSize2, false);
    l_sCam->getCamera2().setRotationMatrix(d.R2, false);
    l_sCam->getCamera2().setPosition(d.T2, false);
    l_sCam->getCamera2().setDistortionCoefficients(d.delta2);
    l_sCam->getCamera2().setIntrinsicCameraMatrix(d.K2, true);
  }
  CameraSettings* cam1;
  if (l_cam1.get())
    cam1 = l_cam1.get();
  else
    cam1 = &l_sCam->getCamera1();
  cam1->setImageSize(d.imSize1, false);
  cam1->setRotationMatrix(d.R1, false);
  cam1->setPosition(d.T1, false);
  cam1->setDistortionCoefficients(d.delta1);
  cam1->setIntrinsicCameraMatrix(d.K1, true);
}

void CalibLoader::setCurrentCalib(const std::string& calibName)
{
  auto it = m_calibs.find(calibName);
  if (it != m_calibs.end())
  {
    sofa::helper::OptionsGroup* t = d_calibNames.beginWriteOnly();
    t->setSelectedItem(it->first);
    d_calibNames.endEdit();

    setCurrentCalib(it->second);
  }
}

void CalibLoader::load(const std::string& filename)
{
  cv::FileStorage* fs;
  try
  {
    fs = new cv::FileStorage(filename, cv::FileStorage::READ);
  }
  catch (cv::Exception& e)
  {
    msg_error("CalibLoader::load()")
        << "cv::FileStorage::open(): File is not a valid XML / YAML file\n"
        << e.what();
    return;
  }

  std::string calibName;
  cv::Mat K1;
  cv::Mat K2;
  cv::Mat delta1;
  cv::Mat delta2;
  cv::Mat imsize1;
  cv::Mat imsize2;
  double error1;
  double error2;
  cv::Mat Rs;
  cv::Mat Ts;
  cv::Mat F;
  cv::Mat E;
  cv::Mat R1;
  cv::Mat T1;
  cv::Mat R2;
  cv::Mat T2;
  double totalError;

  calibName = sofa::helper::system::SetDirectory::GetFileNameWithoutExtension(
      filename.c_str());

  cv::read((*fs)["imsize"], imsize1, cv::Mat(1, 2, CV_32S));
  cv::read((*fs)["K"], K1, cv::Mat(3, 3, CV_64F));
  cv::read((*fs)["R"], R1, cv::Mat::eye(3, 3, CV_64F));
  cv::read((*fs)["t"], T1, cv::Mat(1, 3, CV_64F));
  cv::read((*fs)["delta"], delta1, cv::Mat());
  cv::read((*fs)["error"], error1, -1.0);

  cv::read((*fs)["imsize2"], imsize2, cv::Mat(1, 2, CV_32S));
  cv::read((*fs)["K2"], K2, cv::Mat(3, 3, CV_64F));
  cv::read((*fs)["R2"], R2, cv::Mat::eye(3, 3, CV_64F));
  cv::read((*fs)["t2"], T2, cv::Mat(1, 3, CV_64F));
  cv::read((*fs)["delta2"], delta2, cv::Mat());
  cv::read((*fs)["error2"], error2, -1.0);

  cv::read((*fs)["Rs"], Rs, cv::Mat(3, 3, CV_64F));
  cv::read((*fs)["ts"], Ts, cv::Mat(1, 3, CV_64F));
  cv::read((*fs)["E"], E, cv::Mat(3, 3, CV_64F));
  cv::read((*fs)["F"], F, cv::Mat(3, 3, CV_64F));
  cv::read((*fs)["stereo_error"], totalError, -1.0);

  CalibData c;

  matrix::cvMat2sofaVector(imsize1, c.imSize1);
  matrix::cvMat2sofaMat(K1, c.K1);
  matrix::cvMat2sofaMat(R1, c.R1);
  matrix::cvMat2sofaVector(T1, c.T1);
  matrix::cvMat2sofaVector(delta1, c.delta1);
  c.error1 = error1;

  matrix::cvMat2sofaVector(imsize2, c.imSize2);
  matrix::cvMat2sofaMat(K2, c.K2);
  matrix::cvMat2sofaMat(R2, c.R2);
  matrix::cvMat2sofaVector(T2, c.T2);
  matrix::cvMat2sofaVector(delta2, c.delta2);
  c.error2 = error2;
  matrix::cvMat2sofaMat(Rs, c.Rs);
  matrix::cvMat2sofaVector(Ts, c.Ts);
  matrix::cvMat2sofaMat(F, c.F);
  matrix::cvMat2sofaMat(E, c.E);
  c.totalError = totalError;

  m_calibs[calibName] = c;

  if (calibName == d_calibNames.getValue().getSelectedItem())
    setCurrentCalib(calibName);
  fs->release();
}

void CalibLoader::getAllCalibFiles(const std::string& calibFolder,
                                   std::vector<std::string>& calibFiles)
{
  // Retrieve all calib files (files ending in ".yml" in calibFolder)
  sofa::helper::system::FileSystem::listDirectory(calibFolder, calibFiles,
                                                  "yml");
}

void CalibLoader::parse(sofa::core::objectmodel::BaseObjectDescription* arg)
{
  if (arg->getAttribute("calibDir"))
  {
    sofa::core::objectmodel::DataFileName folder;
    folder.setValue(arg->getAttribute("calibDir"));
    setOptionsGroupToFolder(folder.getFullPath(),
                            arg->getAttribute("calibName"));
  }
  ImplicitDataEngine::parse(arg);
}

void CalibLoader::init()
{
  m_dataTracker.trackData(d_calibNames);
  m_dataTracker.trackData(d_calibFolder);

  addOutput(&d_delta1);
  addOutput(&d_delta2);
  addOutput(&d_K1);
  addOutput(&d_K2);
  addOutput(&d_R1);
  addOutput(&d_T1);
  addOutput(&d_R2);
  addOutput(&d_T2);
  addOutput(&d_error1);
  addOutput(&d_error2);
  addOutput(&d_totalError);
  addOutput(&d_Rs);
  addOutput(&d_Ts);
  addOutput(&d_F);
  addOutput(&d_E);

  if (!l_sCam.get() && !l_cam1.get())
    msg_error(getName() + "::init()") << "Error: No camera link set. "
                                         "Please use attribute 'cam' "
                                         "to define one";
  if (!l_sCam.get()) m_isStereo = false;

  m_isInitialized = true;
  calibFolderChanged();
}

void CalibLoader::calibChanged()
{
  setCurrentCalib(d_calibNames.getValue().getSelectedItem());
}

void CalibLoader::setOptionsGroupToFolder(std::string calibFolder,
                                          std::string calibname)
{
  std::vector<std::string> calibFiles;
  getAllCalibFiles(calibFolder, calibFiles);

  if (calibFiles.empty())
  {
    sofa::helper::OptionsGroup* t = d_calibNames.beginEdit();
    t->setNames(1, "NO_CALIB");
    m_calibs["NO_CALIB"];
    setCurrentCalib("NO_CALIB");
    d_calibNames.endEdit();
  }
  else
  {
    sofa::helper::OptionsGroup* t = d_calibNames.beginEdit();
    t->setNbItems(unsigned(calibFiles.size()));
    unsigned i = 0;
    for (std::string& s : calibFiles)
    {
      t->setItemName(
          i++, sofa::helper::system::SetDirectory::GetFileNameWithoutExtension(
                   s.c_str()));
    }
    if (calibname != "") t->setSelectedItem(calibname);
    if (t->getSelectedItem() != calibname)
    {
      t->setSelectedItem(0);
      calibname = t->getSelectedItem();
    }
    d_calibNames.endEdit();

    // Load calibration files
    for (std::string& f : calibFiles)
      if (canLoad(calibFolder + "/" + f)) load(calibFolder + "/" + f);
    this->setCurrentCalib(calibname);
  }
}

void CalibLoader::calibFolderChanged()
{
  std::string calibname = d_calibNames.getValue().getSelectedItem();
  m_calibs.clear();
  sofa::helper::OptionsGroup* t = d_calibNames.beginEdit();
  t->setNames(0);
  d_calibNames.endEdit();

  const std::string& calibFolder = d_calibFolder.getFullPath();

  setOptionsGroupToFolder(calibFolder, calibname);
}

void CalibLoader::doUpdate()
{
  if (m_dataTracker.hasChanged(d_calibNames)) calibChanged();
  if (m_dataTracker.hasChanged(d_calibFolder)) calibFolderChanged();
}

CalibLoader::CalibData::CalibData(
    const sofa::defaulttype::Matrix3& _K1,
    const sofa::defaulttype::Matrix3& _R1,
    const sofa::defaulttype::Vector3& _T1,
    const sofa::helper::vector<double>& _delta1,
    const sofa::defaulttype::Vec2i _imSize1, double _error1,
    const sofa::defaulttype::Matrix3& _K2,
    const sofa::defaulttype::Matrix3& _R2,
    const sofa::defaulttype::Vector3& _T2,
    const sofa::helper::vector<double>& _delta2,
    const sofa::defaulttype::Vec2i _imSize2, double _error2,
    const sofa::defaulttype::Matrix3& _Rs,
    const sofa::defaulttype::Vector3& _Ts, const sofa::defaulttype::Matrix3& _F,
    const sofa::defaulttype::Matrix3& _E, double _totalError)
    : imSize1(_imSize1),
      K1(_K1),
      R1(_R1),
      T1(_T1),
      delta1(_delta1),
      error1(_error1),
      imSize2(_imSize2),
      K2(_K2),
      R2(_R2),
      T2(_T2),
      delta2(_delta2),
      error2(_error2),
      Rs(_Rs),
      Ts(_Ts),
      F(_F),
      E(_E),
      totalError(_totalError)
{
}

}  // namespace calib
}  // namespace cam
}  // namespace sofacv
