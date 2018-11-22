#include "FeatureDetector.h"
#include <SofaCV/SofaCV.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofacv
{
namespace features
{
SOFA_DECL_CLASS(FeatureDetector)

int FeatureDetectorClass =
    sofa::core::RegisterObject(
        "debug component to visualize images using OpenCV highgui")
        .add<FeatureDetector>();

FeatureDetector::FeatureDetector()
    : ImageFilter(),
      d_detectMode(initData(&d_detectMode, "detectorMode",
                            "if true, does not compute the descriptors")),
      d_mask(initData(&d_mask, cvMat(), "mask",
                      "Mask specifying where to look for keypoints "
                      "(optional). It must be a 8-bit integer matrix with "
                      "non-zero values in the region of interest.")),
      d_detectorType(
          initData(&d_detectorType, "detectorType",
                   "Available feature detection algoritms: FAST, "
                   "MSER, ORB, BRISK, KAZE, AKAZE, BRIEF, BLOB, SIFT, "
                   "SURF, DAISY (the last 3 are only available in "
                   "opencv_contrib)")),
      d_keypoints(initData(&d_keypoints, "keypoints",
                           "output array of cvKeypoints", false)),
      d_descriptors(initData(&d_descriptors, "descriptors",
                             "output cvMat of feature descriptors", false,
                             true))
{
  addAlias(&d_keypoints, "keypoints_out");
  addAlias(&d_descriptors, "descriptors_out");

  sofa::helper::OptionsGroup* t = d_detectMode.beginEdit();
  t->setNames(3, "DETECT_ONLY", "COMPUTE_ONLY", "DETECT_AND_COMPUTE");
  t->setSelectedItem("DETECT_ONLY");
  d_detectMode.endEdit();

  t = d_detectorType.beginEdit();
  t->setNames(DetectorType_COUNT, "FAST", "MSER", "ORB", "BRISK", "KAZE",
              "AKAZE", "BLOB"
#ifdef SOFAOR_OPENCV_CONTRIB_ENABLED
              ,
              "BRIEF", "SIFT", "SURF", "DAISY"
#endif  // SOFAOR_OPENCV_CONTRIB_ENABLED
  );
  t->setSelectedItem("FAST");
  d_detectorType.endEdit();

  m_detectors[FAST] = new FASTDetector(this);
  m_detectors[MSER] = new MSERDetector(this);
  m_detectors[ORB] = new ORBDetector(this);
  m_detectors[BRISK] = new BRISKDetector(this);
  m_detectors[KAZE] = new KAZEDetector(this);
  m_detectors[AKAZE] = new AKAZEDetector(this);
  m_detectors[BLOB] = new SimpleBlobDetector(this);
#ifdef SOFAOR_OPENCV_CONTRIB_ENABLED
  m_detectors[BRIEF] = new BRIEFDetector(this);
  m_detectors[SIFT] = new SIFTDetector(this);
  m_detectors[SURF] = new SURFDetector(this);
  m_detectors[DAISY] = new DAISYDetector(this);
#endif  // SOFAOR_OPENCV_CONTRIB_ENABLED
}

FeatureDetector::~FeatureDetector() {}
void FeatureDetector::init()
{
  addInput(&d_detectMode);
  addInput(&d_detectorType);
  addInput(&d_mask, true);

  for (auto detector : m_detectors) detector->init();

  detectTypeChanged();
  detectModeChanged();

  ImageFilter::init();
}

void FeatureDetector::reinit()
{
  if (m_dataTracker.isDirty(d_detectMode)) detectModeChanged();
  if (m_dataTracker.isDirty(d_detectorType)) detectTypeChanged();

  ImageFilter::reinit();
  update();
}

void FeatureDetector::Update()
{
  sofa::helper::AdvancedTimer::stepBegin("FeatureDetection");
  ImageFilter::Update();
  switch (d_detectMode.getValue().getSelectedId())
  {
    case DETECT_ONLY:
    {
      sofa::helper::vector<cvKeypoint>* vec = d_keypoints.beginEdit();
      vec->clear();
      for (cv::KeyPoint& kp : _v) vec->push_back(cvKeypoint(kp));
      d_keypoints.endEdit();
      break;
    }
    case COMPUTE_ONLY:
    {
      d_descriptors.setValue(_d);
      break;
    }
    case DETECT_AND_COMPUTE:
    {
      sofa::helper::vector<cvKeypoint>* vec = d_keypoints.beginWriteOnly();
      vec->clear();
      for (cv::KeyPoint& kp : _v) vec->push_back(cvKeypoint(kp));
      d_keypoints.endEdit();
      d_descriptors.setValue(_d);
      break;
    }
  }
  sofa::helper::AdvancedTimer::stepEnd("FeatureDetection");
}

void FeatureDetector::applyFilter(const cv::Mat& in, cv::Mat& out, bool debug)
{
  if (in.empty()) return;
  int detect = int(d_detectMode.getValue().getSelectedId());
  if (detect == DETECT_ONLY)
  {
    _v.clear();
    if (!d_keypoints.getValue().empty() && !debug)
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      _v.assign(arr, arr + d_keypoints.getValue().size());
    }
    cv::Mat m = d_mask.getValue();
    if (d_mask.getValue().channels() != 1)
      cv::cvtColor(d_mask.getValue(), m, cv::COLOR_BGR2GRAY);

    m_detectors[d_detectorType.getValue().getSelectedId()]->detect(in, m, _v);
    msg_warning_when(_v.empty(), "FeatureDetector::update()")
        << "No Features detected";
  }
  else if (detect == COMPUTE_ONLY)
  {
    _v.clear();
    if (!d_keypoints.getValue().empty() && !debug)
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      _v.assign(arr, arr + d_keypoints.getValue().size());
    }
    msg_warning_when(_v.empty(), "FeatureDetector::update()")
        << "No Features to describe";
    _d = cv::Mat();
    m_detectors[d_detectorType.getValue().getSelectedId()]->compute(in, _v, _d);
    msg_warning_when(!_d.rows, "FeatureDetector::update()")
        << "Couldn't describe features...";
  }
  else
  {
    if (in.empty()) return;
    _v.clear();
    if (!d_keypoints.getValue().empty() && !debug)
    {
      const cv::KeyPoint* arr =
          dynamic_cast<const cv::KeyPoint*>(d_keypoints.getValue().data());

      _v.assign(arr, arr + d_keypoints.getValue().size());
    }
    _d = cv::Mat();
    m_detectors[d_detectorType.getValue().getSelectedId()]->detectAndCompute(
        in, d_mask.getValue(), _v, _d);
    msg_warning_when(_v.empty(), "FeatureDetector::update()")
        << "Didn't detect any feature...";
    msg_warning_when(!_d.rows, "FeatureDetector::update()")
        << "Couldn't describe features...";
  }
  if (d_outputImage.getValue())
  {
    cv::Mat _in;
    in.copyTo(_in);
    if (_in.depth() == CV_32F)
    {
      _in.convertTo(_in, 0, 255.0);
    }
    if (_in.channels() == 1)
    {
      cv::cvtColor(_in, _in, CV_GRAY2BGR);
    }
    cv::drawKeypoints(_in, _v, _in, cv::Scalar(0, 255, 0), 1);
    out = _in.clone();
  }
  else
    in.copyTo(out);
}

void FeatureDetector::detectModeChanged()
{
  std::cout << "Detector MODE changed to "
            << d_detectMode.getValue().getSelectedItem() << std::endl;
  switch (d_detectMode.getValue().getSelectedId())
  {
    case DETECT_ONLY:
      d_descriptors.setDisplayed(false);
      delInput(&d_keypoints);
      delOutput(&d_descriptors);
      addOutput(&d_keypoints);
      break;
    case COMPUTE_ONLY:
      d_descriptors.setDisplayed(true);
      addInput(&d_keypoints);
      addOutput(&d_descriptors);
      break;
    case DETECT_AND_COMPUTE:
      d_descriptors.setDisplayed(true);
      delInput(&d_keypoints);
      addOutput(&d_keypoints);
      addOutput(&d_descriptors);
      break;
  }
}

void FeatureDetector::detectTypeChanged()
{
  std::cout << "Detector type changed to "
            << d_detectorType.getValue().getSelectedItem() << std::endl;
  for (size_t i = 0; i < DetectorType_COUNT; ++i)
  {
    if (i == d_detectorType.getValue().getSelectedId())
    {
      m_detectors[i]->enable(true);
      unregisterAllData();
      m_detectors[i]->registerData(this);
    }
    else
      m_detectors[i]->enable(false);
  }
}

}  // namespace features
}  // namespace sofacv
