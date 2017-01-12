#include "DetectorOptions.h"
#include "FeatureDetector.h"

namespace sofa
{
namespace OR
{
namespace processor
{
ComputeOpts::ComputeOpts(FeatureDetector* c)
    : d_useProvidedKeypoints(
          c->initData(&d_useProvidedKeypoints, false, "useProvidedKeypoints",
                      "whether or not to perform a feature detection before "
                      "description")),
      d_descriptors(c->initData(&d_descriptors, "descriptors",
                                "output cvMat of feature descriptors", true,
                                true))
{
}

void ComputeOpts::toggleVisible(bool show)
{
  d_useProvidedKeypoints.setDisplayed(show);
  d_descriptors.setDisplayed(show);
}

BaseOpts::~BaseOpts() {}
void BaseOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                      std::vector<cv::KeyPoint>& keypoints)
{
  m_detector->detect(img, keypoints, mask);
}

void BaseOpts::detectAndCompute(const common::cvMat& img,
                                const common::cvMat& mask,
                                std::vector<cv::KeyPoint>& keypoints,
                                common::cvMat& descriptors, bool useKPts)
{
  m_detector->detectAndCompute(img, mask, keypoints, descriptors, useKPts);
}

FASTOpts::FASTOpts(FeatureDetector* c)
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

  m_detector = cv::FastFeatureDetector::create(
      threshold.getValue(), nonmaxsuppression.getValue(),
      int(type.getValue().getSelectedId()));
}

void FASTOpts::toggleVisible(bool show)
{
  threshold.setDisplayed(show);
  nonmaxsuppression.setDisplayed(show);
  type.setDisplayed(show);
}

MSEROpts::MSEROpts(FeatureDetector* c)
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
  m_detector = cv::MSER::create(
      delta.getValue(), minArea.getValue(), maxArea.getValue(),
      maxVariation.getValue(), minDiversity.getValue(), maxEvolution.getValue(),
      areaThreshold.getValue(), minMargin.getValue(), edgeBlurSize.getValue());
}

void MSEROpts::toggleVisible(bool show)
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

ORBOpts::ORBOpts(FeatureDetector* c)
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

  m_detector = cv::ORB::create(nFeatures.getValue(), scaleFactor.getValue(),
                               nLevels.getValue(), edgeThreshold.getValue(),
                               firstLevel.getValue(), WTA_K.getValue(),
                               int(scoreType.getValue().getSelectedId()),
                               patchSize.getValue(), fastThreshold.getValue());
}

void ORBOpts::toggleVisible(bool show)
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

BRISKOpts::BRISKOpts(FeatureDetector* c)
    : threshold(c->initData(&threshold, 30, "BRISKThreshold",
                            "FAST/AGAST detection threshold score.")),
      octaves(c->initData(&octaves, 3, "BRISKOctaves",
                          "detection octaves. Use 0 to do single scale.")),
      npatternScale(c->initData(&npatternScale, 1.0f, "BRISKNpatternScale",
                                "apply this scale to the pattern used for "
                                "sampling the neighbourhood of a keypoint."))
{
  m_detector = cv::BRISK::create(threshold.getValue(), octaves.getValue(),
                                 npatternScale.getValue());
}

void BRISKOpts::toggleVisible(bool show)
{
  threshold.setDisplayed(show);
  octaves.setDisplayed(show);
  npatternScale.setDisplayed(show);
}

KAZEOpts::KAZEOpts(FeatureDetector* c)
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

  m_detector = cv::KAZE::create(extended.getValue(), upright.getValue(),
                                threshold.getValue(), octaves.getValue(),
                                sublevels.getValue(),
                                int(diffusivity.getValue().getSelectedId()));
}

void KAZEOpts::toggleVisible(bool show)
{
  extended.setDisplayed(show);
  upright.setDisplayed(show);
  threshold.setDisplayed(show);
  octaves.setDisplayed(show);
  sublevels.setDisplayed(show);
  diffusivity.setDisplayed(show);
}

AKAZEOpts::AKAZEOpts(FeatureDetector* c)
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

  m_detector = cv::AKAZE::create(
      int(descriptorType.getValue().getSelectedId()), descriptorSize.getValue(),
      descriptorChannels.getValue(), threshold.getValue(), octaves.getValue(),
      sublevels.getValue(), int(diffusivity.getValue().getSelectedId()));
}

void AKAZEOpts::toggleVisible(bool show)
{
  descriptorType.setDisplayed(show);
  descriptorSize.setDisplayed(show);
  descriptorChannels.setDisplayed(show);
  threshold.setDisplayed(show);
  octaves.setDisplayed(show);
  sublevels.setDisplayed(show);
  diffusivity.setDisplayed(show);
}

SIFTOpts::SIFTOpts(FeatureDetector* c)
    : nFeatures(c->initData(&nFeatures, 0, "SIFTNfeatures",
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
  m_detector = cv::xfeatures2d::SIFT::create(
      nFeatures.getValue(), nOctaveLayers.getValue(),
      contrastThreshold.getValue(), edgeThreshold.getValue(), sigma.getValue());
}

void SIFTOpts::toggleVisible(bool show)
{
  nFeatures.setDisplayed(show);
  nOctaveLayers.setDisplayed(show);
  contrastThreshold.setDisplayed(show);
  edgeThreshold.setDisplayed(show);
  sigma.setDisplayed(show);
}

}  // namespace sofa
}  // namespace OR
}  // namespace processor
