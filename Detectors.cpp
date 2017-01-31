#include "Detectors.h"

#include <opencv2/xfeatures2d.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
BaseDetector::~BaseDetector() {}
void BaseDetector::detect(const common::cvMat& img, const common::cvMat& mask,
                          std::vector<cv::KeyPoint>& keypoints)
{
  m_detector->detect(img, keypoints, mask);
}

void BaseDetector::compute(const common::cvMat& img,
                           std::vector<cv::KeyPoint>& keypoints,
                           common::cvMat& descriptors)
{
  m_detector->compute(img, keypoints, descriptors);
}

void BaseDetector::detectAndCompute(const common::cvMat& img,
                                    const common::cvMat& mask,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    common::cvMat& descriptors)
{
  m_detector->detectAndCompute(img, mask, keypoints, descriptors);
}

ShiTomasiDetector::ShiTomasiDetector(core::objectmodel::BaseObject* c)
    : maxCorners(c->initData(&maxCorners, 100, "ShiTomasiMaxCorners",
                            "")),
      qualityLevel(
          c->initData(&qualityLevel, 0.3, "ShiTomasiQualityLevel",
                      "")),
      minDistance(c->initData(&minDistance, 7, "ShiTomasiMinDistance",
                       "")),
      blockSize(c->initData(&blockSize, 7, "ShiTomasiBlockSize",
                       ""))
{
}

void ShiTomasiDetector::init()
{
}

void ShiTomasiDetector::toggleVisible(bool show)
{
  maxCorners.setDisplayed(show);
  qualityLevel.setDisplayed(show);
  minDistance.setDisplayed(show);
  blockSize.setDisplayed(show);
}

FASTDetector::FASTDetector(core::objectmodel::BaseObject* c)
    : threshold(c->initData(&threshold, 0, "FASTThreshold",
                            "threshold on difference between intensity of the "
                            "central pixel and pixels of a circle around this "
                            "pixel.")),
      nonmaxsuppression(
          c->initData(&nonmaxsuppression, false, "FASTNonmaxSuppression",
                      " if true, non-maximum suppression is applied to "
                      "detected corners (keypoints).")),
      type(c->initData(&type, "FASTType",
                       "one of the three neighborhoods as defined in the "
                       "paper: TYPE_9_16, TYPE_7_12, TYPE_5_8"))
{
  sofa::helper::OptionsGroup* t = type.beginEdit();
  t->setNames(3, "TYPE_9_16", "TYPE_7_12", "TYPE_5_8");
  t->setSelectedItem(0);
  type.endEdit();
}

void FASTDetector::init()
{
  m_detector = cv::FastFeatureDetector::create(
      threshold.getValue(), nonmaxsuppression.getValue(),
      int(type.getValue().getSelectedId()));
}

void FASTDetector::toggleVisible(bool show)
{
  threshold.setDisplayed(show);
  nonmaxsuppression.setDisplayed(show);
  type.setDisplayed(show);
}

MSERDetector::MSERDetector(core::objectmodel::BaseObject* c)
    : delta(c->initData(&delta, 5, "MSERDelta",
                        "Compares (sizei - sizei-delta)/sizei-delta")),
      minArea(c->initData(&minArea, 60, "MSERMinArea",
                          "Prune the area which smaller than minArea")),
      maxArea(c->initData(&maxArea, 14400, "MSERMaxArea",
                          "Prune the area which smaller than maxArea")),
      maxVariation(
          c->initData(&maxVariation, 0.25f, "MSERMaxVariation",
                      "Prune the area have simliar size to its children")),
      minDiversity(c->initData(&minDiversity, .2f, "MSERMinDiversity",
                               "For color image, trace back to cut off mser "
                               "with diversity less than min_diversity")),
      maxEvolution(c->initData(&maxEvolution, 200, "MSERMaxEvolution",
                               "For color image, the evolution steps")),
      areaThreshold(c->initData(
          &areaThreshold, 1.01, "MSERAreaThreshold",
          "For color image, the area threshold to cause re-initialize")),
      minMargin(c->initData(&minMargin, 0.003, "MSERMinMargin",
                            "For color image, ignore too small margin")),
      edgeBlurSize(
          c->initData(&edgeBlurSize, 5, "MSEREdgeBlurSize",
                      "For color image, the aperture size for edge blur"))
{
}

void MSERDetector::toggleVisible(bool show)
{
  delta.setDisplayed(show);
  minArea.setDisplayed(show);
  maxArea.setDisplayed(show);
  maxVariation.setDisplayed(show);
  minDiversity.setDisplayed(show);
  maxEvolution.setDisplayed(show);
  areaThreshold.setDisplayed(show);
  minMargin.setDisplayed(show);
  edgeBlurSize.setDisplayed(show);
}

void MSERDetector::init()
{
  m_detector = cv::MSER::create(
      delta.getValue(), minArea.getValue(), maxArea.getValue(),
      maxVariation.getValue(), minDiversity.getValue(), maxEvolution.getValue(),
      areaThreshold.getValue(), minMargin.getValue(), edgeBlurSize.getValue());
}

ORBDetector::ORBDetector(core::objectmodel::BaseObject* c)
    : nFeatures(c->initData(&nFeatures, 500, "ORBNFeatures",
                            "Compares (sizei - sizei-delta)/sizei-delta")),
      scaleFactor(c->initData(&scaleFactor, 1.2f, "scaleFactor",
                              "Prune the area which smaller than minArea")),
      nLevels(c->initData(&nLevels, 8, "ORBNLevels",
                          "Prune the area which smaller than maxArea")),
      edgeThreshold(
          c->initData(&edgeThreshold, 31, "ORBEdgeThreshold",
                      "Prune the area have simliar size to its children")),
      firstLevel(c->initData(&firstLevel, 0, "ORBFirstLevel",
                             "For color image, trace back to cut off mser with "
                             "diversity less than min_diversity")),
      WTA_K(c->initData(&WTA_K, 2, "ORBMaxEvolution",
                        "For color image, the evolution steps")),
      scoreType(c->initData(
          &scoreType, "ORBScoreType",
          "For color image, the area threshold to cause re-initialize")),
      patchSize(c->initData(&patchSize, 31, "ORBPatchSize",
                            "For color image, ignore too small margin")),
      fastThreshold(c->initData(
          &fastThreshold, 20, "ORBFastThreshold",
          "no information provided by opencv for this parameter..."))

{
  sofa::helper::OptionsGroup* t = scoreType.beginEdit();
  t->setNames(2, "HARRIS_SCORE", "FAST_SCORE");
  t->setSelectedItem(0);
  scoreType.endEdit();
}

void ORBDetector::init()
{
  m_detector = cv::ORB::create(nFeatures.getValue(), scaleFactor.getValue(),
                               nLevels.getValue(), edgeThreshold.getValue(),
                               firstLevel.getValue(), WTA_K.getValue(),
                               int(scoreType.getValue().getSelectedId()),
                               patchSize.getValue(), fastThreshold.getValue());
}

void ORBDetector::toggleVisible(bool show)
{
  nFeatures.setDisplayed(show);
  scaleFactor.setDisplayed(show);
  nLevels.setDisplayed(show);
  edgeThreshold.setDisplayed(show);
  firstLevel.setDisplayed(show);
  WTA_K.setDisplayed(show);
  scoreType.setDisplayed(show);
  patchSize.setDisplayed(show);
  fastThreshold.setDisplayed(show);
}

BRISKDetector::BRISKDetector(core::objectmodel::BaseObject* c)
    : threshold(c->initData(&threshold, 30, "BRISKThreshold",
                            "FAST/AGAST detection threshold score.")),
      octaves(c->initData(&octaves, 3, "BRISKOctaves",
                          "detection octaves. Use 0 to do single scale.")),
      npatternScale(c->initData(&npatternScale, 1.0f, "BRISKNpatternScale",
                                "apply this scale to the pattern used for "
                                "sampling the neighbourhood of a keypoint."))
{
}
void BRISKDetector::init()
{
  m_detector = cv::BRISK::create(threshold.getValue(), octaves.getValue(),
                                 npatternScale.getValue());
}

void BRISKDetector::toggleVisible(bool show)
{
  threshold.setDisplayed(show);
  octaves.setDisplayed(show);
  npatternScale.setDisplayed(show);
}

KAZEDetector::KAZEDetector(core::objectmodel::BaseObject* c)
    : extended(c->initData(
          &extended, false, "KAZEExtended",
          "Set to enable extraction of extended (128-byte) descriptor.")),
      upright(c->initData(&upright, false, "KAZEUpright",
                          "Set to enable use of upright descriptors (non "
                          "rotation-invariant).")),
      threshold(c->initData(&threshold, 0.001f, "KAZEThreshold",
                            "Detector response threshold to accept point")),
      octaves(c->initData(&octaves, 4, "KAZEOctaves",
                          "Maximum octave evolution of the image")),
      sublevels(c->initData(&sublevels, 4, "KAZESublevels",
                            "Default number of sublevels per scale level")),
      diffusivity(c->initData(&diffusivity, "KAZEDiffusivity",
                              "Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, "
                              "DIFF_WEICKERT or DIFF_CHARBONNIER"))
{
  sofa::helper::OptionsGroup* t = diffusivity.beginEdit();
  t->setNames(4, "DIFF_PM_G1", "DIFF_PM_G2", "DIFF_WEICKERT",
              "DIFF_CHARBONNIER");
  t->setSelectedItem(3);
  diffusivity.endEdit();
}

void KAZEDetector::init()
{
  m_detector = cv::KAZE::create(extended.getValue(), upright.getValue(),
                                threshold.getValue(), octaves.getValue(),
                                sublevels.getValue(),
                                int(diffusivity.getValue().getSelectedId()));
}
void KAZEDetector::toggleVisible(bool show)
{
  extended.setDisplayed(show);
  upright.setDisplayed(show);
  threshold.setDisplayed(show);
  octaves.setDisplayed(show);
  sublevels.setDisplayed(show);
  diffusivity.setDisplayed(show);
}

AKAZEDetector::AKAZEDetector(core::objectmodel::BaseObject* c)
    : descriptorType(
          c->initData(&descriptorType, "AKAZEDescriptorType",
                      "Type of the extracted descriptor: "
                      "DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT, "
                      "DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.")),
      descriptorSize(
          c->initData(&descriptorSize, 0, "AKAZEDescriptorSize",
                      "Size of the descriptor in bits .0 -> Full size")),
      descriptorChannels(
          c->initData(&descriptorChannels, 3, "AKAZEDescriptorChannels",
                      "Number of channels in the descriptor(1, 2, 3)")),
      threshold(c->initData(&threshold, 0.001f, "AKAZEThreshold",
                            "Detector response threshold to accept point")),
      octaves(c->initData(&octaves, 4, "AKAZEOctaves",
                          "Maximum octave evolution of the image")),
      sublevels(c->initData(&sublevels, 4, "AKAZESublevels",
                            "Default number of sublevels per scale level")),
      diffusivity(c->initData(&diffusivity, "AKAZEDiffusivity",
                              "Diffusivity type.DIFF_PM_G1, DIFF_PM_G2, "
                              "DIFF_WEICKERT or DIFF_CHARBONNIER"))
{
  sofa::helper::OptionsGroup* t = descriptorType.beginEdit();
  t->setNames(4, "DESCRIPTOR_KAZE", "DESCRIPTOR_KAZE_UPRIGHT",
              "DESCRIPTOR_MLDB", "DESCRIPTOR_MLDB_UPRIGHT");
  t->setSelectedItem(2);
  descriptorType.endEdit();

  t = diffusivity.beginEdit();
  t->setNames(4, "DIFF_PM_G1", "DIFF_PM_G2", "DIFF_WEICKERT",
              "DIFF_CHARBONNIER");
  t->setSelectedItem(3);
  diffusivity.endEdit();
}

void AKAZEDetector::init()
{
  m_detector = cv::AKAZE::create(
      int(descriptorType.getValue().getSelectedId()), descriptorSize.getValue(),
      descriptorChannels.getValue(), threshold.getValue(), octaves.getValue(),
      sublevels.getValue(), int(diffusivity.getValue().getSelectedId()));
}
void AKAZEDetector::toggleVisible(bool show)
{
  descriptorType.setDisplayed(show);
  descriptorSize.setDisplayed(show);
  descriptorChannels.setDisplayed(show);
  threshold.setDisplayed(show);
  octaves.setDisplayed(show);
  sublevels.setDisplayed(show);
  diffusivity.setDisplayed(show);
}

SIFTDetector::SIFTDetector(core::objectmodel::BaseObject* c)
    : nFeatures(c->initData(&nFeatures, 0, "SIFTNFeatures",
                            "The number of best features to retain. The "
                            "features are ranked by their scores (measured in "
                            "SIFT algorithm as the local contrast)")),
      nOctaveLayers(c->initData(&nOctaveLayers, 3, "SIFTNOctaveLayers",
                                "The number of layers in each octave. 3 is "
                                "the value used in D. Lowe paper. The number "
                                "of octaves is computed automatically from "
                                "the image resolution.")),
      contrastThreshold(c->initData(
          &contrastThreshold, 0.04, "SIFTContrastThreshold",
          "The contrast threshold used to filter out weak features in "
          "semi-uniform (low-contrast) regions. The larger the threshold, "
          "the less features are produced by the detector.")),
      edgeThreshold(c->initData(
          &edgeThreshold, 10.0, "SIFTEdgeThreshold",
          "The threshold used to filter out edge-like features. Note that "
          "the its meaning is different from the contrastThreshold, i.e. "
          "the larger the edgeThreshold, the less features are filtered "
          "out (more features are retained).")),
      sigma(c->initData(&sigma, 1.6, "SIFTSigma",
                        "The sigma of the Gaussian applied to the input image "
                        "at the octave #0. If your image is captured with a "
                        "weak camera with soft lenses, you might want to "
                        "reduce the number."))
{
}

void SIFTDetector::init()
{
  m_detector = cv::xfeatures2d::SIFT::create(
      nFeatures.getValue(), nOctaveLayers.getValue(),
      contrastThreshold.getValue(), edgeThreshold.getValue(), sigma.getValue());
}
void SIFTDetector::toggleVisible(bool show)
{
  nFeatures.setDisplayed(show);
  nOctaveLayers.setDisplayed(show);
  contrastThreshold.setDisplayed(show);
  edgeThreshold.setDisplayed(show);
  sigma.setDisplayed(show);
}

SURFDetector::SURFDetector(core::objectmodel::BaseObject* c)
    : threshold(c->initData(
          &threshold, 100.0, "SURFThreshold",
          "Hessian threshold for keypoint detector. Only features, whose "
          "hessian is larger than hessianThreshold are retained by the "
          "detector. Therefore, the larger the value, the less keypoints you "
          "will get. A good default value could be from 300 to 500, depending "
          "on the image contrast.")),
      nOctaves(c->initData(
          &nOctaves, 4, "SURFNOctaves",
          "The number of a gaussian pyramid octaves that the detector uses. It "
          "is set to 4 by default. If you want to get very large features, use "
          "the larger value. If you want just small features, decrease it.)")),
      nOctaveLayers(c->initData(&nOctaveLayers, 3, "SURFNOctaveLayers",
                                "The number of images within each octave of a "
                                "gaussian pyramid. It is set to 2 by "
                                "default.")),
      extended(c->initData(&extended, false, "SURFExtended",
                           " - 0 means that the basic descriptors (64 elements "
                           "each) shall be computed\n"
                           " - 1 means that the extended descriptors (128 "
                           "elements each) shall be computed")),
      upright(c->initData(
          &upright, false, "SURFUpright",
          " - 0 means that detector computes orientation of each feature.\n"
          " - 1 means that the orientation is not computed (which is much, "
          "much "
          "faster). For example, if you match images from a stereo pair, or do "
          "image stitching, the matched features likely have very similar "
          "angles, and you can speed up feature extraction by setting "
          "upright=1."))
{
}

void SURFDetector::init()
{
  m_detector = cv::xfeatures2d::SURF::create(
      threshold.getValue(), nOctaves.getValue(), nOctaveLayers.getValue(),
      extended.getValue(), upright.getValue());
}
void SURFDetector::toggleVisible(bool show)
{
  threshold.setDisplayed(show);
  nOctaves.setDisplayed(show);
  nOctaveLayers.setDisplayed(show);
  extended.setDisplayed(show);
  upright.setDisplayed(show);
}

BRIEFDetector::BRIEFDetector(core::objectmodel::BaseObject* c)
    : bytes(c->initData(&bytes, 32, "BRIEFBytes",
                        "length of the descriptor in bytes, valid values are: "
                        "16, 32 (default) or 64 ")),
      use_orientation(c->initData(
          &use_orientation, false, "BRIEFUseOrientation",
          "sample patterns using keypoints orientation, disabled by default."))
{
}

void BRIEFDetector::init()
{
  m_detector = cv::xfeatures2d::BriefDescriptorExtractor::create(
      bytes.getValue(), use_orientation.getValue());
}
void BRIEFDetector::toggleVisible(bool show)
{
  bytes.setDisplayed(show);
  use_orientation.setDisplayed(show);
}

DAISYDetector::DAISYDetector(core::objectmodel::BaseObject* c)
    : radius(c->initData(&radius, 15.0f, "DAISYRadius",
                         "radius of the descriptor at the initial scale")),
      q_radius(c->initData(&q_radius, 3, "DAISYQRadius",
                           "amount of radial range division quantity")),
      q_theta(c->initData(&q_theta, 8, "DAISYQTheta",
                          "amount of angular range division quantity")),
      q_hist(c->initData(
          &q_hist, 8, "DAISYQHist",
          "amount of gradient orientations range division quantity")),
      norm(c->initData(
          &norm, "DAISYNorm",
          "choose descriptors normalization type, where DAISY::NRM_NONE will "
          "not do any normalization (default), DAISY::NRM_PARTIAL mean that "
          "histograms are normalized independently for L2 norm equal to 1.0, "
          "DAISY::NRM_FULL mean that descriptors are normalized for L2 norm "
          "equal to 1.0, DAISY::NRM_SIFT mean that descriptors are normalized "
          "for L2 norm equal to 1.0 but no individual one is bigger than 0.154 "
          "as in SIFT")),
      H(c->initData(&H, common::cvMat(), "DAISYH",
                    "optional 3x3 homography matrix used to warp the grid of "
                    "daisy but sampling keypoints remains unwarped on image")),
      interpolation(c->initData(&interpolation, true, "DAISYInterpolation",
                                "switch to disable interpolation for speed "
                                "improvement at minor quality loss")),
      use_orientation(c->initData(
          &use_orientation, false, "DAISYUseOrientation",
          "sample patterns using keypoints orientation, disabled by default."))
{
  sofa::helper::OptionsGroup* t = norm.beginEdit();
  t->setNames(4, "NRM_NONE", "NRM_PARTIAL", "NRM_FULL", "NRM_SIFT");
  t->setSelectedItem("NRM_NONE");
  norm.endEdit();
}

void DAISYDetector::init()
{
  m_detector = cv::xfeatures2d::DAISY::create(
      radius.getValue(), q_radius.getValue(), q_theta.getValue(),
      q_hist.getValue(), int(norm.getValue().getSelectedId()), H.getValue(),
      interpolation.getValue(), use_orientation.getValue());
}
void DAISYDetector::toggleVisible(bool show)
{
  radius.setDisplayed(show);
  q_radius.setDisplayed(show);
  q_theta.setDisplayed(show);
  q_hist.setDisplayed(show);
  norm.setDisplayed(show);
  H.setDisplayed(show);
  interpolation.setDisplayed(show);
  use_orientation.setDisplayed(show);
}

}  // namespace sofa
}  // namespace OR
}  // namespace processor
