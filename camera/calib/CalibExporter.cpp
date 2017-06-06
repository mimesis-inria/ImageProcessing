#include "CalibExporter.h"

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
SOFA_DECL_CLASS(CalibExporter)

int CalibExporterClass =
		core::RegisterObject("Mono / stereo Camera calibration settings exporter")
				.add<CalibExporter>();

CalibExporter::CalibExporter()
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
														 "directory in which calibration will be stored")),
			d_calibName(
					initData(&d_calibName, "fileName",
									 "Calibration file name (timestamp if not specified")),
			d_nSteps(initData(&d_nSteps, (unsigned)0, "nSteps",
												"number of steps between each export (0 means no "
												"export during animation")),
			d_exportType(initData(&d_exportType, "exportType",
														"specify when export should happen (BEGIN, END, "
														"STEP). if STEP, specify export frequency with "
														"nSteps. Default is END")),
			d_activate(initData(&d_activate, true, "active",
													"if false, nothing will be exported")),
			m_stepCounter(0),
			m_isStereo(true)
{
	f_listening.setValue(true);
	sofa::helper::OptionsGroup* t = d_exportType.beginEdit();
	t->setNames(3, "BEGIN", "END", "STEP");
	t->setSelectedItem("END");
	d_exportType.endEdit();
}

CalibExporter::~CalibExporter() {}

void CalibExporter::init()
{
	m_stepCounter = 0;

	if (!l_sCam.get() && !l_cam1.get())
		msg_error(getName() + "::init()") << "Error: No camera link set. "
																				 "Please use attribute 'cam' "
																				 "to define one";
	if (!l_sCam.get()) m_isStereo = false;
	exportCalib(d_calibName.getValue());
}

void CalibExporter::update()
{
	++m_stepCounter;
	switch (d_exportType.getValue().getSelectedId())
	{
		case 0:  // BEGIN
			if (m_stepCounter == 1)
			{
				exportCalib(d_calibName.getValue());
			}
			break;
		case 2:  // STEP
			if (m_stepCounter % d_nSteps.getValue() == 0)
			{
				exportCalib(std::to_string(m_stepCounter) + d_calibName.getValue());
			}
			break;
		default:
			break;
	}
}

void CalibExporter::cleanup()
{
	if (d_exportType.getValue().getSelectedId() == 1)
		exportCalib(d_calibName.getValue());
}

void CalibExporter::export_cam(cv::Mat KL, cv::Mat TL, cv::Mat RL, cv::FileStorage fs, double e1, cv::Mat dvL, cv::Mat resL)
{
	fs.writeComment("\nimage size in pixels (w, h)");
	fs << "imsize" << resL;
	fs.writeComment("\nIntrinsic matrix");
	fs << "K" << KL;
	fs.writeComment("\ncamera's orientation in world coordinates");
	fs << "R" << RL;
	fs.writeComment("\ncamera's optical center position in world coordinates");
	fs << "t" << TL;
	fs.writeComment("\nif a non-linear model is used, distortion coefficients");
	fs << "delta" << dvL;
	fs.writeComment("\nreprojection error");
	fs << "error" << e1;
}

void CalibExporter::exportCalib(const std::string& calibFile)
{
	if (!canExport(d_calibFolder.getValue(), calibFile)) return;

	cv::FileStorage fs(d_calibFolder.getValue() + "/" + calibFile,
										 cv::FileStorage::WRITE);
	if (m_isStereo)
	{
		cv::Mat E, F, Rs, ts, RL, TL, RR, TR, dvL, dvR, KR, KL, resL, resR;
		double e1, e2, es;
		e1 = e2 = es = 0.0;

		common::matrix::sofaMat2cvMat(
				l_sCam->getCamera1().getIntrinsicCameraMatrix(), KL);
		common::matrix::sofaMat2cvMat(l_sCam->getCamera1().getRotationMatrix(), RL);
		common::matrix::sofaVector2cvMat(l_sCam->getCamera1().getPosition(), TL);
		common::matrix::sofaVector2cvMat(
				l_sCam->getCamera1().getDistortionCoefficients(), dvL);
		common::matrix::sofaVector2cvMat(l_sCam->getCamera1().getImageSize(), resL);

		common::matrix::sofaMat2cvMat(
				l_sCam->getCamera2().getIntrinsicCameraMatrix(), KR);
		common::matrix::sofaMat2cvMat(l_sCam->getCamera2().getRotationMatrix(), RR);
		common::matrix::sofaVector2cvMat(l_sCam->getCamera2().getPosition(), TR);
		common::matrix::sofaVector2cvMat(
				l_sCam->getCamera2().getDistortionCoefficients(), dvR);
		common::matrix::sofaVector2cvMat(l_sCam->getCamera2().getImageSize(), resR);

		common::matrix::sofaMat2cvMat(l_sCam->getEssentialMatrix(), E);
		common::matrix::sofaMat2cvMat(l_sCam->getFundamentalMatrix(), F);
//		common::matrix::sofaMat2cvMat(l_sCam->getRotationMatrix(), Rs);
//		common::matrix::sofaVector2cvMat(l_sCam->getTranslationVector(), ts);

		export_cam(KL, TL, RL, fs, e1, dvL, resL);

		fs.writeComment("\nSame for second camera if any");
		fs << "imsize2" << resR;
		fs << "K2" << KR;
		fs << "R2" << RR;
		fs << "t2" << TR;
		fs << "delta2" << dvR;
		fs << "error2" << e2;

		fs.writeComment("\ntriangulated reprojection error");
		fs << "stereo_error" << es;
		fs.writeComment("\nEssential matrix");
		fs << "E" << E;
		fs.writeComment("\nFundamental matrix");
		fs << "F" << F;
		fs.writeComment(
				"\nSecond Camera's orientation in the 1st camera's coordinates");
		fs << "Rs" << Rs;
		fs.writeComment(
				"\nSecond Camera's optical center position in the 1st camera's "
				"coordinates");
		fs << "ts" << ts;
	}
	else
	{
		cv::Mat R, t, dvL, KL, res;
		double e = 0.0;

		common::matrix::sofaMat2cvMat(l_cam1->getRotationMatrix(), R);
		common::matrix::sofaVector2cvMat(l_cam1->getPosition(), t);

		common::matrix::sofaMat2cvMat(l_cam1->getIntrinsicCameraMatrix(), KL);
		common::matrix::sofaVector2cvMat(l_cam1->getDistortionCoefficients(), dvL);
		common::matrix::sofaVector2cvMat(l_cam1->getImageSize(), res);

		export_cam(KL, t, R, fs, e, dvL, res);
	}

	fs.release();
	std::cout << "Write Done." << std::endl;
}

bool CalibExporter::canExport(const std::string& fileDir,
															const std::string& fileName) const
{
	if (fileName.empty())
	{
		msg_error("CalibExporter::canLoad()")
				<< "Error: CalibExporter was given an empty calibName";
		return false;
	}

	std::ifstream f(std::string(fileDir + "/" + fileName).c_str(),
									std::ios::out | std::ios::trunc);

	// -- Check if file is readable:
	if (!f.good())
	{
		msg_error("CalibExporter::canLoad()")
				<< "Error: Cannot read file '" << fileDir + "/" + fileName << "'.";
		return false;
	}
	f.close();
	return true;
}

}  // namespace processor
}  // namespace OR
}  // namespace sofa
