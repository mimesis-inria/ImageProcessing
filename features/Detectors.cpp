#include "Detectors.h"

#include <opencv2/xfeatures2d.hpp>

namespace sofacv
{
namespace features
{
BaseDetector::~BaseDetector() {}
void BaseDetector::detect(const cvMat &img, const cvMat &mask,
                          std::vector<cv::KeyPoint> &keypoints)
{
    m_detector->detect(img, keypoints, mask);
}

void BaseDetector::compute(const cvMat &img,
                           std::vector<cv::KeyPoint> &keypoints,
                           cvMat &descriptors)
{
    m_detector->compute(img, keypoints, descriptors);
}

void BaseDetector::detectAndCompute(const cvMat &img,
                                    const cvMat &mask,
                                    std::vector<cv::KeyPoint> &keypoints,
                                    cvMat &descriptors)
{
    m_detector->detectAndCompute(img, mask, keypoints, descriptors);
}

SimpleBlobDetector::SimpleBlobDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      thresholdStep(c->initData(&thresholdStep, 10, "BLOBthresholdStep", "")),
      minThreshold(c->initData(&minThreshold, 10, "BLOBminThreshold", "")),
      maxThreshold(c->initData(&maxThreshold, 200, "BLOBmaxThreshold", "")),
      minDistBetweenBlobs(c->initData(&minDistBetweenBlobs, 5
                                      , "BLOBminDistBetweenBlobs", "")),
      filterByColor(c->initData(&filterByColor, true, "BLOBfilterByColor", "")),
      blobColor(c->initData(&blobColor, 0, "BLOBblobColor", "")),
      filterByArea(c->initData(&filterByArea, true, "BLOBfilterByArea", "")),
      minArea(c->initData(&minArea, 1500, "BLOBminArea", "")),
      filterByCircularity(c->initData(&filterByCircularity, true,
                                      "BLOBfilterByCircularity", "")),
      minCircularity(
          c->initData(&minCircularity, 0.1, "BLOBminCircularity", "")),
      filterByConvexity(
          c->initData(&filterByConvexity, true, "BLOBfilterByConvexity", "")),
      minConvexity(c->initData(&minConvexity, 0.87, "BLOBminConvexity", "")),
      filterByInertia(
          c->initData(&filterByInertia, true, "BLOBfilterByInertia", "")),
      minInertiaRatio(
          c->initData(&minInertiaRatio, 0.1, "BLOBminInertiaRatio", ""))
{
}

void SimpleBlobDetector::enable(bool show)
{
    if (show)
    {
        m_obj->addInput(&thresholdStep);
        m_obj->addInput(&minThreshold);
        m_obj->addInput(&maxThreshold);
        m_obj->addInput(&minDistBetweenBlobs);
        m_obj->addInput(&filterByColor);
        m_obj->addInput(&blobColor);
        m_obj->addInput(&filterByArea);
        m_obj->addInput(&minArea);
        m_obj->addInput(&filterByCircularity);
        m_obj->addInput(&minCircularity);
        m_obj->addInput(&filterByConvexity);
        m_obj->addInput(&minConvexity);
        m_obj->addInput(&filterByInertia);
        m_obj->addInput(&minInertiaRatio);
    }
    else
    {
        m_obj->delInput(&thresholdStep);
        m_obj->delInput(&minThreshold);
        m_obj->delInput(&maxThreshold);
        m_obj->delInput(&minDistBetweenBlobs);
        m_obj->delInput(&filterByColor);
        m_obj->delInput(&blobColor);
        m_obj->delInput(&filterByArea);
        m_obj->delInput(&minArea);
        m_obj->delInput(&filterByCircularity);
        m_obj->delInput(&minCircularity);
        m_obj->delInput(&filterByConvexity);
        m_obj->delInput(&minConvexity);
        m_obj->delInput(&filterByInertia);
        m_obj->delInput(&minInertiaRatio);
    }
    minThreshold.setDisplayed(show);
    maxThreshold.setDisplayed(show);
    filterByArea.setDisplayed(show);
    minArea.setDisplayed(show);
    filterByCircularity.setDisplayed(show);
    minCircularity.setDisplayed(show);
    filterByConvexity.setDisplayed(show);
    minConvexity.setDisplayed(show);
    filterByInertia.setDisplayed(show);
    minInertiaRatio.setDisplayed(show);
}

void SimpleBlobDetector::init()
{
}

void SimpleBlobDetector::registerData(ImageFilter *parent)
{
    parent->registerData(&minThreshold, 0, 255, 1);
    parent->registerData(&thresholdStep, 0, 255, 1);
    parent->registerData(&minThreshold, 0, 255, 1);
    parent->registerData(&maxThreshold, 0, 255, 1);
    parent->registerData(&minDistBetweenBlobs, 0, 255, 1);
    parent->registerData(&filterByColor);
    parent->registerData(&blobColor, 0, 255, 1);
    parent->registerData(&filterByArea);
    parent->registerData(&minArea, 0, 3000, 2);
    parent->registerData(&filterByCircularity);
    parent->registerData(&minCircularity, 0.0, 1.0, 0.001);
    parent->registerData(&filterByConvexity);
    parent->registerData(&minConvexity, 0.0, 1.0, 0.01);
    parent->registerData(&filterByInertia);
    parent->registerData(&minInertiaRatio, 0.0, 1.0, 0.01);
}

void SimpleBlobDetector::detect(const cvMat &img,
                                const cvMat &mask,
                                std::vector<cv::KeyPoint> &keypoints)
{
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
    params.thresholdStep = thresholdStep.getValue();
    params.minThreshold = minThreshold.getValue();
    params.maxThreshold = maxThreshold.getValue();

    // Merging
    params.minDistBetweenBlobs = minDistBetweenBlobs.getValue();

    // Filter by Color.
    params.filterByColor = filterByColor.getValue();
    params.blobColor = blobColor.getValue();

    // Filter by Area.
    params.filterByArea = filterByArea.getValue();
    params.minArea = minArea.getValue();

    // Filter by Circularity
    params.filterByCircularity = filterByCircularity.getValue();
    params.minCircularity = minCircularity.getValue();

    // Filter by Convexity
    params.filterByConvexity = filterByConvexity.getValue();
    params.minConvexity = minConvexity.getValue();

    // Filter by Inertia
    params.filterByInertia = filterByInertia.getValue();
    params.minInertiaRatio = minInertiaRatio.getValue();

#if CV_MAJOR_VERSION < 3  // If you are using OpenCV 2

    // Set up detector with params
    cv::SimpleBlobDetector detector(params);

    // You can use the detector this way
    detector.detect(img, keypoints);

#else

    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector =
            cv::SimpleBlobDetector::create(params);

    // SimpleBlobDetector::create creates a smart pointer.
    // So you need to use arrow ( ->) instead of dot ( . )
    detector->detect(img, keypoints, mask);

#endif
}

void SimpleBlobDetector::compute(const cvMat &,
                                 std::vector<cv::KeyPoint> &, cvMat &)
{
    msg_error("SimpleBlobDetector::compute()")
            << "SimpleBlob is detectOnly. descriptors won't be computed.";
}

void SimpleBlobDetector::detectAndCompute(const cvMat &img,
                                          const cvMat &mask,
                                          std::vector<cv::KeyPoint> &kpts,
                                          cvMat &)
{
    msg_warning("SimpleBlobDetector::detectAndCompute()")
            << "SimpleBlob is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
}

ShiTomasiDetector::ShiTomasiDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      maxCorners(c->initData(&maxCorners, 100, "ShiTomasiMaxCorners", "")),
      qualityLevel(
          c->initData(&qualityLevel, 0.3, "ShiTomasiQualityLevel", "")),
      minDistance(c->initData(&minDistance, 7, "ShiTomasiMinDistance", "")),
      blockSize(c->initData(&blockSize, 7, "ShiTomasiBlockSize", ""))
{
}

void ShiTomasiDetector::init() {}

void ShiTomasiDetector::registerData(ImageFilter *parent)
{
    parent->registerData(&maxCorners, 0, 255, 1);
    parent->registerData(&qualityLevel, 0.0, 1.0, 0.01);
    parent->registerData(&minDistance, 0, 20, 1);
    parent->registerData(&blockSize, 0, 20, 1);
}

void ShiTomasiDetector::detect(const cvMat &, const cvMat &,
                               std::vector<cv::KeyPoint> &)
{
    msg_error("ShiTomasiDetector::detect()") << "Not Implemented Yet";
}

void ShiTomasiDetector::compute(const cvMat &,
                                std::vector<cv::KeyPoint> &, cvMat &)
{
    msg_error("FASTDetector::compute()")
            << "FAST is detectOnly. descriptors won't be computed.";
}

void ShiTomasiDetector::detectAndCompute(const cvMat &img,
                                         const cvMat &mask,
                                         std::vector<cv::KeyPoint> &kpts,
                                         cvMat &)
{
    msg_warning("FASTDetector::detectAndCompute()")
            << "FAST is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
}
void ShiTomasiDetector::enable(bool show)
{
    if (show)
    {
        m_obj->addInput(&maxCorners);
        m_obj->addInput(&qualityLevel);
        m_obj->addInput(&minDistance);
        m_obj->addInput(&blockSize);
    }
    else
    {
        m_obj->delInput(&maxCorners);
        m_obj->delInput(&qualityLevel);
        m_obj->delInput(&minDistance);
        m_obj->delInput(&blockSize);
    }
    maxCorners.setDisplayed(show);
    qualityLevel.setDisplayed(show);
    minDistance.setDisplayed(show);
    blockSize.setDisplayed(show);
}

FASTDetector::FASTDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      threshold(c->initData(&threshold, 0, "FASTThreshold",
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
    sofa::helper::OptionsGroup *t = type.beginEdit();
    t->setNames(3, "TYPE_9_16", "TYPE_7_12", "TYPE_5_8");
    t->setSelectedItem(0);
    type.endEdit();
}

void FASTDetector::init() {}

void FASTDetector::registerData(ImageFilter *parent)
{
    parent->registerData(&threshold, 0, 255, 1);
    parent->registerData(&nonmaxsuppression);
    parent->registerData(&type);
}

void FASTDetector::compute(const cvMat &, std::vector<cv::KeyPoint> &,
                           cvMat &)
{
    msg_error("FASTDetector::compute()")
            << "FAST is detectOnly. descriptors won't be computed.";
}

void FASTDetector::detectAndCompute(const cvMat &img,
                                    const cvMat &mask,
                                    std::vector<cv::KeyPoint> &kpts,
                                    cvMat &)
{
    msg_warning("FASTDetector::detectAndCompute()")
            << "FAST is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
}

void FASTDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::FastFeatureDetector::create(
                    threshold.getValue(), nonmaxsuppression.getValue(),
                    int(type.getValue().getSelectedId()));
        m_obj->addInput(&threshold);
        m_obj->addInput(&nonmaxsuppression);
        m_obj->addInput(&type);
    }
    else
    {
        m_obj->delInput(&threshold);
        m_obj->delInput(&nonmaxsuppression);
        m_obj->delInput(&type);
    }
    threshold.setDisplayed(show);
    nonmaxsuppression.setDisplayed(show);
    type.setDisplayed(show);
}

MSERDetector::MSERDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      delta(c->initData(&delta, 5, "MSERDelta",
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
                        "For color image, the area threshold to cause "
                        "re-initialize")),
      minMargin(c->initData(&minMargin, 0.003, "MSERMinMargin",
                            "For color image, ignore too small margin")),
      edgeBlurSize(
          c->initData(&edgeBlurSize, 5, "MSEREdgeBlurSize",
                      "For color image, the aperture size for edge blur"))
{
}

void MSERDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::MSER::create(
                    delta.getValue(), minArea.getValue(), maxArea.getValue(),
                    double(maxVariation.getValue())
                    , double(minDiversity.getValue()),
                    maxEvolution.getValue(), areaThreshold.getValue()
                    , minMargin.getValue(),
                    edgeBlurSize.getValue());
        m_obj->addInput(&delta);
        m_obj->addInput(&minArea);
        m_obj->addInput(&maxArea);
        m_obj->addInput(&maxVariation);
        m_obj->addInput(&minDiversity);
        m_obj->addInput(&maxEvolution);
        m_obj->addInput(&areaThreshold);
        m_obj->addInput(&minMargin);
        m_obj->addInput(&edgeBlurSize);
    }
    else
    {
        m_obj->delInput(&delta);
        m_obj->delInput(&minArea);
        m_obj->delInput(&maxArea);
        m_obj->delInput(&maxVariation);
        m_obj->delInput(&minDiversity);
        m_obj->delInput(&maxEvolution);
        m_obj->delInput(&areaThreshold);
        m_obj->delInput(&minMargin);
        m_obj->delInput(&edgeBlurSize);
    }
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

void MSERDetector::init() {}

void MSERDetector::registerData(ImageFilter *parent)
{
    parent->registerData(&delta, 0, 10, 1);
    parent->registerData(&minArea, 0, 255, 1);
    parent->registerData(&maxArea, 10000, 20000, 200);

    parent->registerData(&maxVariation, 0.0f, 1.0f, 0.01f);
    parent->registerData(&minDiversity, 0.0f, 1.0f, 0.01f);
    parent->registerData(&maxEvolution, 0, 400, 1);
    parent->registerData(&areaThreshold, 1.0, 2.0, 0.01);
    parent->registerData(&minMargin, 0.0, 0.1, 0.0001);
    parent->registerData(&edgeBlurSize, 0, 10, 1);
}

void MSERDetector::compute(const cvMat &, std::vector<cv::KeyPoint> &,
                           cvMat &)
{
    msg_error("MSERDetector::detectAndCompute()")
            << "MSER is detectOnly. descriptors won't be computed.";
}

void MSERDetector::detectAndCompute(const cvMat &img,
                                    const cvMat &mask,
                                    std::vector<cv::KeyPoint> &kpts,
                                    cvMat &)
{
    msg_warning("MSERDetector::detectAndCompute()")
            << "MSER is detectOnly. descriptors won't be computed.";
    detect(img, mask, kpts);
}

ORBDetector::ORBDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      nFeatures(c->initData(&nFeatures, 500, "ORBNFeatures",
                            "Compares (sizei - sizei-delta)/sizei-delta")),
      scaleFactor(c->initData(&scaleFactor, 1.2f, "ORBScaleFactor",
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
                    &scoreType, "ORBScoreType", "For color image, the area "
                    "threshold to cause re-initialize")),
      patchSize(c->initData(&patchSize, 31, "ORBPatchSize",
                            "For color image, ignore too small margin")),
      fastThreshold(c->initData(
                        &fastThreshold, 20, "ORBFastThreshold", "no information"
                        " provided by opencv for this parameter..."))

{
    sofa::helper::OptionsGroup *t = scoreType.beginEdit();
    t->setNames(2, "HARRIS_SCORE", "FAST_SCORE");
    t->setSelectedItem(0);
    scoreType.endEdit();
}

void ORBDetector::init() {}

void ORBDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}

void ORBDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::ORB::create(
                    nFeatures.getValue(), scaleFactor.getValue()
                    , nLevels.getValue(), edgeThreshold.getValue()
                    , firstLevel.getValue(), WTA_K.getValue(),
                    int(scoreType.getValue().getSelectedId()),
                    patchSize.getValue(), fastThreshold.getValue());
        m_obj->addInput(&nFeatures);
        m_obj->addInput(&scaleFactor);
        m_obj->addInput(&nLevels);
        m_obj->addInput(&edgeThreshold);
        m_obj->addInput(&firstLevel);
        m_obj->addInput(&WTA_K);
        m_obj->addInput(&scoreType);
        m_obj->addInput(&patchSize);
        m_obj->addInput(&fastThreshold);
    }
    else
    {
        m_obj->delInput(&nFeatures);
        m_obj->delInput(&scaleFactor);
        m_obj->delInput(&nLevels);
        m_obj->delInput(&edgeThreshold);
        m_obj->delInput(&firstLevel);
        m_obj->delInput(&WTA_K);
        m_obj->delInput(&scoreType);
        m_obj->delInput(&patchSize);
        m_obj->delInput(&fastThreshold);
    }
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

BRISKDetector::BRISKDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      threshold(c->initData(&threshold, 30, "BRISKThreshold",
                            "FAST/AGAST detection threshold score.")),
      octaves(c->initData(&octaves, 3, "BRISKOctaves",
                          "detection octaves. Use 0 to do single scale.")),
      npatternScale(c->initData(&npatternScale, 1.0f, "BRISKNpatternScale",
                                "apply this scale to the pattern used for "
                                "sampling the neighbourhood of a keypoint."))
{
}
void BRISKDetector::init() {}

void BRISKDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}

void BRISKDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::BRISK::create(threshold.getValue(), octaves.getValue(),
                                       npatternScale.getValue());
        m_obj->addInput(&threshold);
        m_obj->addInput(&octaves);
        m_obj->addInput(&npatternScale);
    }
    else
    {
        m_obj->delInput(&threshold);
        m_obj->delInput(&octaves);
        m_obj->delInput(&npatternScale);
    }
    threshold.setDisplayed(show);
    octaves.setDisplayed(show);
    npatternScale.setDisplayed(show);
}

KAZEDetector::KAZEDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      extended(c->initData(
                   &extended, false, "KAZEExtended", "Set to enable extraction "
                   "of extended (128-byte) descriptor.")),
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
    sofa::helper::OptionsGroup *t = diffusivity.beginEdit();
    t->setNames(4, "DIFF_PM_G1", "DIFF_PM_G2", "DIFF_WEICKERT",
                "DIFF_CHARBONNIER");
    t->setSelectedItem(3);
    diffusivity.endEdit();
}

void KAZEDetector::init() {}

void KAZEDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}
void KAZEDetector::enable(bool show)
{
    if (show)
    {
        m_detector =
                cv::KAZE::create(extended.getValue(), upright.getValue(),
                                 threshold.getValue(), octaves.getValue(),
                                 sublevels.getValue(),
                                 int(diffusivity.getValue().getSelectedId()));
        m_obj->addInput(&extended);
        m_obj->addInput(&upright);
        m_obj->addInput(&threshold);
        m_obj->addInput(&octaves);
        m_obj->addInput(&sublevels);
        m_obj->addInput(&diffusivity);
    }
    else
    {
        m_obj->delInput(&extended);
        m_obj->delInput(&upright);
        m_obj->delInput(&threshold);
        m_obj->delInput(&octaves);
        m_obj->delInput(&sublevels);
        m_obj->delInput(&diffusivity);
    }

    extended.setDisplayed(show);
    upright.setDisplayed(show);
    threshold.setDisplayed(show);
    octaves.setDisplayed(show);
    sublevels.setDisplayed(show);
    diffusivity.setDisplayed(show);
}

AKAZEDetector::AKAZEDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      descriptorType(
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
    sofa::helper::OptionsGroup *t = descriptorType.beginEdit();
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

void AKAZEDetector::init() {}

void AKAZEDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}
void AKAZEDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::AKAZE::create(
                    int(descriptorType.getValue().getSelectedId()),
                    descriptorSize.getValue(), descriptorChannels.getValue(),
                    threshold.getValue(), octaves.getValue(),
                    sublevels.getValue(),
                    int(diffusivity.getValue().getSelectedId()));
        m_obj->addInput(&descriptorType);
        m_obj->addInput(&descriptorSize);
        m_obj->addInput(&descriptorChannels);
        m_obj->addInput(&threshold);
        m_obj->addInput(&octaves);
        m_obj->addInput(&sublevels);
        m_obj->addInput(&diffusivity);
    }
    else
    {
        m_obj->delInput(&descriptorType);
        m_obj->delInput(&descriptorSize);
        m_obj->delInput(&descriptorChannels);
        m_obj->delInput(&threshold);
        m_obj->delInput(&octaves);
        m_obj->delInput(&sublevels);
        m_obj->delInput(&diffusivity);
    }
    descriptorType.setDisplayed(show);
    descriptorSize.setDisplayed(show);
    descriptorChannels.setDisplayed(show);
    threshold.setDisplayed(show);
    octaves.setDisplayed(show);
    sublevels.setDisplayed(show);
    diffusivity.setDisplayed(show);
}

#ifdef SOFACV_OPENCV_CONTRIB_ENABLED

BRIEFDetector::BRIEFDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      bytes(c->initData(&bytes, 32, "BRIEFBytes",
                        "length of the descriptor in bytes, valid values are: "
                        "16, 32 (default) or 64 ")),
      use_orientation(c->initData(&use_orientation, false,
                                  "BRIEFUseOrientation",
                                  "sample patterns using keypoints "
                                  "orientation, disabled by default."))
{
}

void BRIEFDetector::init() {}

void BRIEFDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}

void BRIEFDetector::detect(const cvMat &, const cvMat &,
                           std::vector<cv::KeyPoint> &)
{
    msg_error("BRIEFDetector::detect()")
            << "BRIEF is computeOnly. keypoints must be provided.";
}

void BRIEFDetector::detectAndCompute(const cvMat &,
                                     const cvMat &,
                                     std::vector<cv::KeyPoint> &,
                                     cvMat &)
{
    msg_error("BRIEFDetector::detectAndCompute()")
            << "BRIEF is computeOnly. Please provide keypoints and set "
               "DetectMode to COMPUTE_ONLY";
}
void BRIEFDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::xfeatures2d::BriefDescriptorExtractor::create(
                    bytes.getValue(), use_orientation.getValue());
        m_obj->addInput(&bytes);
        m_obj->addInput(&use_orientation);
    }
    else
    {
        m_obj->delInput(&bytes);
        m_obj->delInput(&use_orientation);
    }
    bytes.setDisplayed(show);
    use_orientation.setDisplayed(show);
}

SIFTDetector::SIFTDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      nFeatures(c->initData(&nFeatures, 0, "SIFTNFeatures",
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
                            "The contrast threshold used to filter out weak "
                            "features in semi-uniform (low-contrast) regions. "
                            "The larger the threshold, the less features are "
                            "produced by the detector.")),
      edgeThreshold(c->initData(
                        &edgeThreshold, 10.0, "SIFTEdgeThreshold",
                        "The threshold used to filter out edge-like features. "
                        "Note that the its meaning is different from the "
                        "contrastThreshold, i.e. the larger the edgeThreshold, "
                        "the less features are filtered out (more features are "
                        "retained).")),
      sigma(c->initData(&sigma, 1.6, "SIFTSigma",
                        "The sigma of the Gaussian applied to the input image "
                        "at the octave #0. If your image is captured with a "
                        "weak camera with soft lenses, you might want to "
                        "reduce the number."))
{
}

void SIFTDetector::init() {}

void SIFTDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}
void SIFTDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::xfeatures2d::SIFT::create(
                    nFeatures.getValue(), nOctaveLayers.getValue(),
                    contrastThreshold.getValue(), edgeThreshold.getValue(),
                    sigma.getValue());
        m_obj->addInput(&nFeatures);
        m_obj->addInput(&nOctaveLayers);
        m_obj->addInput(&contrastThreshold);
        m_obj->addInput(&edgeThreshold);
        m_obj->addInput(&sigma);
    }
    else
    {
        m_obj->delInput(&nFeatures);
        m_obj->delInput(&nOctaveLayers);
        m_obj->delInput(&contrastThreshold);
        m_obj->delInput(&edgeThreshold);
        m_obj->delInput(&sigma);
    }
    nFeatures.setDisplayed(show);
    nOctaveLayers.setDisplayed(show);
    contrastThreshold.setDisplayed(show);
    edgeThreshold.setDisplayed(show);
    sigma.setDisplayed(show);
}

SURFDetector::SURFDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      threshold(c->initData(
                    &threshold, 100.0, "SURFThreshold",
                    "Hessian threshold for keypoint detector. Only features, "
                    "whose hessian is larger than hessianThreshold are retained"
                    " by the detector. Therefore, the larger the value, the "
                    "less keypoints you will get. A good default value could be"
                    " from 300 to 500, depending on the image contrast.")),
      nOctaves(c->initData(&nOctaves, 4, "SURFNOctaves",
                           "The number of a gaussian pyramid octaves that "
                           "the detector uses. It "
                           "is set to 4 by default. If you want to get very "
                           "large features, use "
                           "the larger value. If you want just small "
                           "features, decrease it.)")),
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
                  " - 0 : computes orientation of each feature.\n"
                  " - 1 : orientation is not computed (which is much, much "
                  "faster). For example, if you match images from a stereo "
                  "pair, or do image stitching, the matched features likely "
                  "have very similar angles, and you can speed up feature "
                  "extraction by setting upright=1."))
{
}

void SURFDetector::init() {}

void SURFDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}
void SURFDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::xfeatures2d::SURF::create(
                    threshold.getValue(), nOctaves.getValue(),
                    nOctaveLayers.getValue(), extended.getValue(),
                    upright.getValue());
        m_obj->addInput(&threshold);
        m_obj->addInput(&nOctaves);
        m_obj->addInput(&nOctaveLayers);
        m_obj->addInput(&extended);
        m_obj->addInput(&upright);
    }
    else
    {
        m_obj->delInput(&threshold);
        m_obj->delInput(&nOctaves);
        m_obj->delInput(&nOctaveLayers);
        m_obj->delInput(&extended);
        m_obj->delInput(&upright);
    }
    threshold.setDisplayed(show);
    nOctaves.setDisplayed(show);
    nOctaveLayers.setDisplayed(show);
    extended.setDisplayed(show);
    upright.setDisplayed(show);
}

DAISYDetector::DAISYDetector(sofa::core::DataEngine *c)
    : BaseDetector(c),
      radius(c->initData(&radius, 15.0f, "DAISYRadius",
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
               "choose descriptors normalization type, where DAISY::NRM_NONE "
               "will not do any normalization (default), DAISY::NRM_PARTIAL "
               "mean that histograms are normalized independently for L2 norm "
               "equal to 1.0, DAISY::NRM_FULL mean that descriptors are "
               "normalized for L2 norm equal to 1.0, DAISY::NRM_SIFT mean that "
               "descriptors are normalized for L2 norm equal to 1.0 but no "
               "individual one is bigger than 0.154 as in SIFT")),
      H(c->initData(&H, cvMat(), "DAISYH",
                    "optional 3x3 homography matrix used to warp the grid of "
                    "daisy but sampling keypoints remains unwarped on image")),
      interpolation(c->initData(&interpolation, true, "DAISYInterpolation",
                                "switch to disable interpolation for speed "
                                "improvement at minor quality loss")),
      use_orientation(c->initData(&use_orientation, false,
                                  "DAISYUseOrientation",
                                  "sample patterns using keypoints "
                                  "orientation, disabled by default."))
{
    sofa::helper::OptionsGroup *t = norm.beginEdit();
    t->setNames(4, "NRM_NONE", "NRM_PARTIAL", "NRM_FULL", "NRM_SIFT");
    t->setSelectedItem("NRM_NONE");
    norm.endEdit();
}

void DAISYDetector::init() {}

void DAISYDetector::registerData(ImageFilter *)
{
    // TODO: find optimal range of values
}

void DAISYDetector::detect(const cvMat &, const cvMat &,
                           std::vector<cv::KeyPoint> &)
{
    msg_error("DAISYDetector::detect()")
            << "DAISY is computeOnly. keypoints must be provided.";
}

void DAISYDetector::detectAndCompute(const cvMat &,
                                     const cvMat &,
                                     std::vector<cv::KeyPoint> &,
                                     cvMat &)
{
    msg_error("DAISYDetector::detectAndCompute()")
            << "DAISY is computeOnly. Please provide keypoints and set "
               "DetectMode to COMPUTE_ONLY";
}

void DAISYDetector::enable(bool show)
{
    if (show)
    {
        m_detector = cv::xfeatures2d::DAISY::create(
                    radius.getValue(), q_radius.getValue(), q_theta.getValue(),
                    q_hist.getValue(), int(norm.getValue().getSelectedId()),
                    H.getValue(), interpolation.getValue(),
                    use_orientation.getValue());
        m_obj->addInput(&radius);
        m_obj->addInput(&q_radius);
        m_obj->addInput(&q_theta);
        m_obj->addInput(&q_hist);
        m_obj->addInput(&norm);
        m_obj->addInput(&H);
        m_obj->addInput(&interpolation);
        m_obj->addInput(&use_orientation);
    }
    else
    {
        m_obj->delInput(&radius);
        m_obj->delInput(&q_radius);
        m_obj->delInput(&q_theta);
        m_obj->delInput(&q_hist);
        m_obj->delInput(&norm);
        m_obj->delInput(&H);
        m_obj->delInput(&interpolation);
        m_obj->delInput(&use_orientation);
    }
    radius.setDisplayed(show);
    q_radius.setDisplayed(show);
    q_theta.setDisplayed(show);
    q_hist.setDisplayed(show);
    norm.setDisplayed(show);
    H.setDisplayed(show);
    interpolation.setDisplayed(show);
    use_orientation.setDisplayed(show);
}

#endif  // SOFACV_OPENCV_CONTRIB_ENABLED

}  // namespace features
}  // namespace sofacv
