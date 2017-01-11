#include "DetectorOptions.h"
#include "FeatureDetector.h"

namespace sofa
{
namespace OR
{
namespace processor
{
BaseOpts::~BaseOpts() {}
FASTOpts::FASTOpts(FeatureDetector* c)
    : threshold(c->initData(&threshold, 0, "threshold",
                            "threshold on difference between intensity of the "
                            "central pixel and pixels of a circle around this "
                            "pixel.")),
      nonmaxsuppression(
          c->initData(&nonmaxsuppression, false, "nonmaxSuppression",
                      " if true, non-maximum suppression is applied to "
                      "detected corners (keypoints).")),
      type(c->initData(&type, "type",
                       "one of the three neighborhoods as defined in the "
                       "paper: TYPE_9_16, TYPE_7_12, TYPE_5_8"))
{
  sofa::helper::OptionsGroup* t = type.beginEdit();
  t->setNames(3, "TYPE_9_16", "TYPE_7_12", "TYPE_5_8");
  t->setSelectedItem(0);
  type.endEdit();
}

void FASTOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                      std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(
      threshold.getValue(), nonmaxsuppression.getValue(),
      int(type.getValue().getSelectedId()));

  detector->detect(img, keypoints, mask);
}

MSEROpts::MSEROpts(FeatureDetector* c)
    : delta(c->initData(&delta, 5, "delta",
                        "Compares (sizei - sizei-delta)/sizei-delta")),
      minArea(c->initData(&minArea, 60, "minArea",
                          "Prune the area which smaller than minArea")),
      maxArea(c->initData(&maxArea, 14400, "maxArea",
                          "Prune the area which smaller than maxArea")),
      maxVariation(
          c->initData(&maxVariation, 0.25f, "maxVariation",
                      "Prune the area have simliar size to its children")),
      minDiversity(c->initData(&minDiversity, .2f, "minDiversity",
                               "For color image, trace back to cut off mser "
                               "with diversity less than min_diversity")),
      maxEvolution(c->initData(&maxEvolution, 200, "maxEvolution",
                               "For color image, the evolution steps")),
      areaThreshold(c->initData(
          &areaThreshold, 1.01, "areaThreshold",
          "For color image, the area threshold to cause re-initialize")),
      minMargin(c->initData(&minMargin, 0.003, "minMargin",
                            "For color image, ignore too small margin")),
      edgeBlurSize(
          c->initData(&edgeBlurSize, 5, "edgeBlurSize",
                      "For color image, the aperture size for edge blur"))
{
}

void MSEROpts::detect(const common::cvMat& img, const common::cvMat& mask,
                      std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::MSER> detector = cv::MSER::create(
      delta.getValue(), minArea.getValue(), maxArea.getValue(),
      maxVariation.getValue(), minDiversity.getValue(), maxEvolution.getValue(),
      areaThreshold.getValue(), minMargin.getValue(), edgeBlurSize.getValue());

  detector->detect(img, keypoints, mask);
}

ORBOpts::ORBOpts(FeatureDetector* c)
    : nFeatures(c->initData(&nFeatures, 500, "nFeatures",
                            "Compares (sizei - sizei-delta)/sizei-delta")),
      scaleFactor(c->initData(&scaleFactor, 1.2f, "scaleFactor",
                              "Prune the area which smaller than minArea")),
      nLevels(c->initData(&nLevels, 8, "nLevels",
                          "Prune the area which smaller than maxArea")),
      edgeThreshold(
          c->initData(&edgeThreshold, 31, "edgeThreshold",
                      "Prune the area have simliar size to its children")),
      firstLevel(c->initData(&firstLevel, 0, "firstLevel",
                             "For color image, trace back to cut off mser with "
                             "diversity less than min_diversity")),
      WTA_K(c->initData(&WTA_K, 2, "maxEvolution",
                        "For color image, the evolution steps")),
      scoreType(c->initData(
          &scoreType, "scoreType",
          "For color image, the area threshold to cause re-initialize")),
      patchSize(c->initData(&patchSize, 31, "patchSize",
                            "For color image, ignore too small margin")),
      fastThreshold(c->initData(
          &fastThreshold, 20, "fastThreshold",
          "no information provided by opencv for this parameter..."))

{
  sofa::helper::OptionsGroup* t = scoreType.beginEdit();
  t->setNames(2, "HARRIS_SCORE", "FAST_SCORE");
  t->setSelectedItem(0);
  scoreType.endEdit();
}

void ORBOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                     std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::ORB> detector = cv::ORB::create(
      nFeatures.getValue(), scaleFactor.getValue(), nLevels.getValue(),
      edgeThreshold.getValue(), firstLevel.getValue(), WTA_K.getValue(),
      int(scoreType.getValue().getSelectedId()), patchSize.getValue(),
      fastThreshold.getValue());

  detector->detect(img, keypoints, mask);
}

BRISKOpts::BRISKOpts(FeatureDetector* c)
    : threshold(c->initData(&threshold, 30, "threshold",
                            "FAST/AGAST detection threshold score.")),
      octaves(c->initData(&octaves, 3, "octaves",
                          "detection octaves. Use 0 to do single scale.")),
      npatternScale(c->initData(&npatternScale, 1.0f, "npatternScale",
                                "apply this scale to the pattern used for "
                                "sampling the neighbourhood of a keypoint."))
{
}

void BRISKOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                       std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::BRISK> detector = cv::BRISK::create(
      threshold.getValue(), octaves.getValue(), npatternScale.getValue());

  detector->detect(img, keypoints, mask);
}

KAZEOpts::KAZEOpts(FeatureDetector* c)
    : extended(c->initData(
          &extended, false, "extended",
          "Set to enable extraction of extended (128-byte) descriptor.")),
      upright(c->initData(&upright, false, "upright",
                          "Set to enable use of upright descriptors (non "
                          "rotation-invariant).")),
      threshold(c->initData(&threshold, 0.001f, "threshold",
                            "Detector response threshold to accept point")),
      octaves(c->initData(&octaves, 4, "octaves",
                          "Maximum octave evolution of the image")),
      sublevels(c->initData(&sublevels, 4, "sublevels",
                            "Default number of sublevels per scale level")),
      diffusivity(c->initData(&diffusivity, "diffusivity",
                              "Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, "
                              "DIFF_WEICKERT or DIFF_CHARBONNIER"))
{
  sofa::helper::OptionsGroup* t = diffusivity.beginEdit();
  t->setNames(4, "DIFF_PM_G1", "DIFF_PM_G2", "DIFF_WEICKERT",
              "DIFF_CHARBONNIER");
  t->setSelectedItem(3);
  diffusivity.endEdit();
}

void KAZEOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                      std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::Feature2D> detector = cv::KAZE::create(
      extended.getValue(), upright.getValue(), threshold.getValue(),
      octaves.getValue(), sublevels.getValue(),
      int(diffusivity.getValue().getSelectedId()));
  detector->detect(img, keypoints, mask);
}

AKAZEOpts::AKAZEOpts(FeatureDetector* c)
    : descriptorType(
          c->initData(&descriptorType, "descriptorType",
                      "Type of the extracted descriptor: "
                      "DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT, "
                      "DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.")),
      descriptorSize(
          c->initData(&descriptorSize, 0, "descriptorSize",
                      "Size of the descriptor in bits .0 -> Full size")),
      descriptorChannels(
          c->initData(&descriptorChannels, 3, "descriptorChannels",
                      "Number of channels in the descriptor(1, 2, 3)")),
      threshold(c->initData(&threshold, 0.001f, "threshold",
                            "Detector response threshold to accept point")),
      octaves(c->initData(&octaves, 4, "octaves",
                          "Maximum octave evolution of the image")),
      sublevels(c->initData(&sublevels, 4, "sublevels",
                            "Default number of sublevels per scale level")),
      diffusivity(c->initData(&diffusivity, "diffusivity",
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

void AKAZEOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                       std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::Feature2D> detector = cv::AKAZE::create(
      int(descriptorType.getValue().getSelectedId()), descriptorSize.getValue(),
      descriptorChannels.getValue(), threshold.getValue(), octaves.getValue(),
      sublevels.getValue(), int(diffusivity.getValue().getSelectedId()));
  detector->detect(img, keypoints, mask);
}

SIFTOpts::SIFTOpts(FeatureDetector* c)
    : nFeatures(c->initData(&nFeatures, 0, "nfeatures",
                            "The number of best features to retain. The "
                            "features are ranked by their scores (measured in "
                            "SIFT algorithm as the local contrast)")),
      nOctaveLayers(c->initData(&nOctaveLayers, 3, "nOctaveLayers",
                                "The number of layers in each octave. 3 is "
                                "the value used in D. Lowe paper. The number "
                                "of octaves is computed automatically from "
                                "the image resolution.")),
      contrastThreshold(c->initData(
          &contrastThreshold, 0.04, "contrastThreshold",
          "The contrast threshold used to filter out weak features in "
          "semi-uniform (low-contrast) regions. The larger the threshold, "
          "the less features are produced by the detector.")),
      edgeThreshold(c->initData(
          &edgeThreshold, 10.0, "edgeThreshold",
          "The threshold used to filter out edge-like features. Note that "
          "the its meaning is different from the contrastThreshold, i.e. "
          "the larger the edgeThreshold, the less features are filtered "
          "out (more features are retained).")),
      sigma(c->initData(&sigma, 1.6, "sigma",
                        "The sigma of the Gaussian applied to the input image "
                        "at the octave #0. If your image is captured with a "
                        "weak camera with soft lenses, you might want to "
                        "reduce the number."))
{
}

void SIFTOpts::detect(const common::cvMat& img, const common::cvMat& mask,
                      std::vector<cv::KeyPoint>& keypoints)
{
  cv::Ptr<cv::Feature2D> detector = cv::xfeatures2d::SIFT::create(
      nFeatures.getValue(), nOctaveLayers.getValue(),
      contrastThreshold.getValue(), edgeThreshold.getValue(), sigma.getValue());

  detector->detect(img, keypoints, mask);
}

}  // namespace sofa
}  // namespace OR
}  // namespace processor
