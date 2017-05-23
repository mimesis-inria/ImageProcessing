#include "CalibLoader.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileSystem.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <cstring>
#include <fstream>
#include <string>

#include <opencv2/core/persistence.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
SOFA_DECL_CLASS(CalibLoader)

int CalibLoaderClass =
    core::RegisterObject("Mono / stereo Camera calibration settings loader")
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
      d_isStereo(initData(&d_isStereo, "isStereo",
                          "set to true if the given calib file has stereo data",
                          true, true)),
      d_projMat1(initData(
          &d_projMat1, "projMat1",
          "projection matrix for the first camera (or default if mono)", true,
          true)),
      d_distCoefs1(initData(
          &d_distCoefs1, "distCoefs1",
          "distortion coefficients for the first camera (or default if mono)",
          true, true)),
      d_error1(initData(&d_error1, "error1",
                        "first camera's reprojection error", true, true)),
      d_projMat2(initData(&d_projMat2, "projMat2",
                          "projection matrix for the second camera (if any)",
                          true, true)),
      d_distCoefs2(
          initData(&d_distCoefs2, "distCoefs2",
                   "distortion coefficients for the second camera (if any)",
                   true, true)),
      d_error2(initData(&d_error2, "error2",
                        "second camera's reprojection error (if any)", true,
                        true)),
      d_R(initData(&d_R, "R", "rotation matrix (extrinsic parameter)", true,
                   true)),
      d_T(initData(&d_T, "T", "translation vector (extrinsic parameter)", true,
                   true)),
      d_F(initData(&d_F, "F", "fundamental matrix (extrinsic parameter)", true,
                   true)),
      d_totalError(initData(&d_totalError, "totalError",
                            "total stereo reprojection error", true, true))
{
  f_listening.setValue(true);

  // Set of aliases to allow implicit initialization of data
  addAlias(&d_projMat1, "projMat1_out");
  addAlias(&d_projMat1, "projMat");

  addAlias(&d_projMat2, "projMat2_out");

  addAlias(&d_projMat1, "distCoefs1_out");
  addAlias(&d_projMat1, "distCoefs");

  addAlias(&d_projMat2, "distCoefs2_out");

  addAlias(&d_error1, "error");
  addAlias(&d_error1, "error1_out");

  addAlias(&d_error2, "error2_out");

  addAlias(&d_R, "R_out");
  addAlias(&d_T, "T_out");
  addAlias(&d_F, "F_out");
  addAlias(&d_totalError, "totalError_out");
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

  if (!sofa::helper::system::DataRepository.findFile(sfilename))
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
	d_projMat1.setValue(d.projMat1);
  d_projMat2.setValue(d.projMat2);
  d_distCoefs1.setValue(d.distCoefs1);
  d_distCoefs2.setValue(d.distCoefs2);
  d_error1.setValue(d.error1);
  d_error2.setValue(d.error2);
  d_R.setValue(d.R);
  d_T.setValue(d.T);
  d_F.setValue(d.F);
  d_totalError.setValue(d.totalError);

	if (d_projMat2.getValue().empty() || !d_isStereo.getValue())
		d_isStereo.setValue(false);
	else
	{
		l_cam2->setDistortionCoefficients(d.distCoefs2);
		l_cam2->setIntrinsicCameraMatrix(d.projMat2, false);
		l_sCam->setFundamentalMatrix(d.F);
		l_sCam->setRotationMatrix(d.R);
		l_sCam->setTranslationVector(d.T);
		l_cam2->setPosition(d.T, false);
		l_cam2->setRotationMatrix(d.R, true);
	}
	l_cam1->setDistortionCoefficients(d.distCoefs1);
	l_cam1->setIntrinsicCameraMatrix(d.projMat1);
	l_cam1->setRotationMatrix(d.R);
	l_cam1->setPosition(d.T);
}

void CalibLoader::setCurrentCalib(const std::string& calibName)
{
  auto it = m_calibs.find(calibName);
  if (it != m_calibs.end())
  {
    helper::OptionsGroup* t = d_calibNames.beginWriteOnly();
    t->setSelectedItem(it->first);
    d_calibNames.endEdit();
    setCurrentCalib(it->second);
  }
}

void CalibLoader::load(const std::string& filename)
{
  cv::FileStorage* fs = NULL;
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
	cv::Mat cameraMatrix1;
	cv::Mat cameraMatrix2;
	cv::Mat distVec1;
	cv::Mat distVec2;
  double error1;
  double error2;
	cv::Mat R;
	cv::Mat T;
	cv::Mat F;
  double totalError;

  calibName = helper::system::SetDirectory::GetFileNameWithoutExtension(
      filename.c_str());

	cv::read((*fs)["cam_mats_left"], cameraMatrix1, cv::Mat());
  cv::read((*fs)["cam_mats_right"], cameraMatrix2, cv::Mat());
  cv::read((*fs)["error_left"], error1, -1.0);
  cv::read((*fs)["error_right"], error2, -1.0);
  cv::read((*fs)["dist_coefs_left"], distVec1, cv::Mat());
  cv::read((*fs)["dist_coefs_right"], distVec2, cv::Mat());
  cv::read((*fs)["rot_mat"], R, cv::Mat());
  cv::read((*fs)["trans_vec"], T, cv::Mat());
  cv::read((*fs)["f_mat"], F, cv::Mat());
  cv::read((*fs)["avg_error"], totalError, -1.0);

  CalibData c;

	common::matrix::cvMat2sofaMat(cameraMatrix1, c.projMat1);
	common::matrix::cvMat2sofaVector(distVec1, c.distCoefs1);
  c.error1 = error1;
	common::matrix::cvMat2sofaMat(cameraMatrix2, c.projMat2);
  common::matrix::cvMat2sofaVector(distVec2, c.distCoefs2);
  c.error2 = error2;
  common::matrix::cvMat2sofaMat(R, c.R);
	common::matrix::cvMat2sofaVector(T, c.T);
  common::matrix::cvMat2sofaMat(F, c.F);
  c.totalError = totalError;

	m_calibs[calibName] = c;

  if (calibName == d_calibNames.getValue().getSelectedItem())
    setCurrentCalib(calibName);
  fs->release();
}

std::string CalibLoader::getPathToCalibs()
{
  std::string currentDir = helper::system::SetDirectory::GetCurrentDir();
  // If exists, add calibFolder to current dir
  if (d_calibFolder.getValue() != "")
  {
    std::string path = d_calibFolder.getValue();

    if (!path.empty())
      currentDir = helper::system::SetDirectory::GetRelativeFromDir(
          path.c_str(), currentDir.c_str());
  }
  return currentDir;
}

void CalibLoader::getAllCalibFiles(std::vector<std::string>& calibFiles)
{
  // Retrieve all calib files (files ending in ".yml" in calibFolder)
  helper::system::FileSystem::listDirectory(getPathToCalibs(), calibFiles,
                                            "yml");
}

void CalibLoader::init()
{
  d_calibNames.beginWriteOnly()->setSelectedItemToDefault();
  d_calibNames.endEdit();

  addDataCallback(&d_calibNames,
									(ImplicitDataEngine::DataCallback)&CalibLoader::calibChanged);
	addDataCallback(
			&d_calibFolder,
			(ImplicitDataEngine::DataCallback)&CalibLoader::calibFolderChanged);

  addOutput(&d_isStereo);
  addOutput(&d_distCoefs1);
  addOutput(&d_distCoefs2);
  addOutput(&d_projMat1);
  addOutput(&d_projMat2);
  addOutput(&d_error1);
  addOutput(&d_error2);
  addOutput(&d_totalError);
  addOutput(&d_R);
  addOutput(&d_T);
  addOutput(&d_F);

	if (!l_cam1.get())
		msg_error(getName() + "::init()") << "Error: No camera link set. "
																				 "Please use attribute 'cam' "
																				 "to define one";

  calibFolderChanged(NULL);
}

void CalibLoader::calibChanged(core::objectmodel::BaseObject*)
{
  setCurrentCalib(d_calibNames.getValue().getSelectedItem());
}

void CalibLoader::calibFolderChanged(core::objectmodel::BaseObject*)
{
  std::string calibname = d_calibNames.getValue().getSelectedItem();

  m_calibs.clear();
  helper::OptionsGroup* t = d_calibNames.beginEdit();
  t->setNames(0);
  d_calibNames.endEdit();

  std::string currentDir = getPathToCalibs();
  std::vector<std::string> calibFiles;
  getAllCalibFiles(calibFiles);

  if (calibFiles.empty())
  {
    helper::OptionsGroup* t = d_calibNames.beginEdit();
    t->setNames(1, "NO_CALIB");
    m_calibs["NO_CALIB"];
    setCurrentCalib("NO_CALIB");
    d_calibNames.endEdit();
  }
  else
  {
    helper::OptionsGroup* t = d_calibNames.beginEdit();
    t->setNbItems(unsigned(calibFiles.size()));
    unsigned i = 0;
    for (std::string& s : calibFiles)
    {
      t->setItemName(
          i++,
          helper::system::SetDirectory::GetFileNameWithoutExtension(s.c_str()));
    }
    (calibname != "") ? (t->setSelectedItem(calibname))
                      : t->setSelectedItemToDefault();
    d_calibNames.endEdit();

    // Load calibration files
    for (std::string& f : calibFiles)
      if (canLoad(currentDir + "/" + f)) load(currentDir + "/" + f);
  }
}

void CalibLoader::update() {}
}  // namespace processor
}  // namespace OR
}  // namespace sofa
