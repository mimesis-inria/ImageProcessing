/******************************************************************************
*       SOFAOR, SOFA plugin for the Operating Room, development version       *
*                        (c) 2017 INRIA, MIMESIS Team                         *
*                                                                             *
* This program is a free software; you can redistribute it and/or modify it   *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 1.0 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Bruno Marques and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact-mimesis@inria.fr                               *
******************************************************************************/

#include "Matchers.h"

#include <opencv2/features2d.hpp>

namespace sofaor
{
namespace processor
{
namespace features
{
BaseMatcher::~BaseMatcher() {}
void BaseMatcher::knnMatch(const common::cvMat& queryDescriptors,
                           const common::cvMat& trainDescriptors,
                           std::vector<std::vector<cv::DMatch> >& matches,
                           int k, const common::cvMat& mask)
{
  m_matcher->knnMatch(queryDescriptors, trainDescriptors, matches, k, mask);
}

void BaseMatcher::radiusMatch(const common::cvMat& queryDescriptors,
                              const common::cvMat& trainDescriptors,
                              std::vector<std::vector<cv::DMatch> >& matches,
                              float maxDistance, const common::cvMat& mask)
{
  m_matcher->radiusMatch(queryDescriptors, trainDescriptors, matches,
                         maxDistance, mask);
}

BFMatcher::BFMatcher(sofa::core::objectmodel::BaseObject* c)
    : normType(c->initData(
          &normType, "BFNormType",
          "One of NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2. L1 and L2 "
          "norms are preferable choices for SIFT and SURF descriptors, "
          "NORM_HAMMING should be used with ORB, BRISK and BRIEF, "
          "NORM_HAMMING2 should be used with ORB when WTA_K==3 or 4 (see "
          "ORB::ORB constructor description")),
      crossCheck(c->initData(
          &crossCheck, false, "BFcrossCheck",
          "If it is false, this is will be default BFMatcher behaviour when it "
          "finds the k nearest neighbors for each query descriptor. If "
          "crossCheck==true, then the knnMatch() method with k=1 will only "
          "return pairs (i,j) such that for i-th query descriptor the j-th "
          "descriptor in the matcher's collection is the nearest and vice "
          "versa, i.e. the BFMatcher will only return consistent pairs. Such "
          "technique usually produces best results with minimal number of "
          "outliers when there are enough matches. This is alternative to the "
          "ratio test, used by D. Lowe in SIFT paper."))
{
  sofa::helper::OptionsGroup* t = normType.beginEdit();
  t->setNames(4, "NORM_L1", "NORM_L2", "NORM_HAMMING", "NORM_HAMMING2");
  t->setSelectedItem(1);
  normType.endEdit();
}

const int BFMatcher::cvNorms[4]{cv::NORM_L1, cv::NORM_L2, cv::NORM_HAMMING,
                                cv::NORM_HAMMING2};

void BFMatcher::toggleVisible(bool show)
{
  normType.setDisplayed(show);
  crossCheck.setDisplayed(show);
}

void BFMatcher::init()
{
  m_matcher = new cv::BFMatcher(cvNorms[normType.getValue().getSelectedId()],
                                crossCheck.getValue());
}

FlannMatcher::FlannMatcher(sofa::core::objectmodel::BaseObject* c)
    : indexParamsType(c->initData(&indexParamsType, "indexParams",
                                  "AUTOTUNED, COMPOSITE, "
                                  "HIERARCHICAL_CLUSTERING, KDTREE, KMEANS, "
                                  "LINEAR, LSH, SAVED"))
{
  sofa::helper::OptionsGroup* t = indexParamsType.beginEdit();
  t->setNames(IndexParamsType_COUNT, "AUTOTUNED", "COMPOSITE",
              "HIERARCHICAL_CLUSTERING", "KDTREE", "KMEANS", "LINEAR", "LSH",
              "SAVED");
  t->setSelectedItem("KDTREE");
  indexParamsType.endEdit();

  m_AllIndexParams[AUTOTUNED] = new AutotunedIndexParams(c);
  m_AllIndexParams[COMPOSITE] = new CompositeIndexParams(c);
  m_AllIndexParams[HIERARCHICAL_CLUSTERING] =
      new HierarchicalClusteringIndexParams(c);
  m_AllIndexParams[KDTREE] = new KDTreeIndexParams(c);
  m_AllIndexParams[KMEANS] = new KMeansIndexParams(c);
  m_AllIndexParams[LINEAR] = new LinearIndexParams(c);
  m_AllIndexParams[LSH] = new LshIndexParams(c);
  m_AllIndexParams[SAVED] = new SavedIndexParams(c);

  m_searchParams = new SearchParams(c);
	m_indexParams = m_AllIndexParams[KDTREE];
}

void FlannMatcher::init()
{
  for (unsigned i = 0; i < IndexParamsType_COUNT; ++i)
  {
    if (i == indexParamsType.getValue().getSelectedId())
    {
      m_indexParams = m_AllIndexParams[i];
      m_indexParams->toggleVisible(true);
    }
    else
      m_AllIndexParams[i]->toggleVisible(false);
  }
  m_matcher = new cv::FlannBasedMatcher(m_indexParams->getIndexParams(),
                                        m_searchParams->getSearchParams());
}

void FlannMatcher::toggleVisible(bool show)
{
  indexParamsType.setDisplayed(show);
	m_indexParams->toggleVisible(show);
}

FlannMatcher::AutotunedIndexParams::AutotunedIndexParams(
		sofa::core::objectmodel::BaseObject* c)
    : target_precision(
          c->initData(&target_precision, 0.8f, "AUTOTUNEDTargetPrecision", "")),
      build_weight(
          c->initData(&build_weight, 0.01f, "AUTOTUNEDBuildWeight", "")),
      memory_weight(
          c->initData(&memory_weight, 0.0f, "AUTOTUNEDMemoryWeight", "")),
      sample_fraction(
          c->initData(&sample_fraction, 0.1f, "AUTOTUNEDSampleFraction", ""))
{
}

cv::Ptr<cv::flann::IndexParams>
FlannMatcher::AutotunedIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::AutotunedIndexParams>(
      target_precision.getValue(), build_weight.getValue(),
      memory_weight.getValue(), sample_fraction.getValue());
}

void FlannMatcher::AutotunedIndexParams::toggleVisible(bool show)
{
  target_precision.setDisplayed(show);
  build_weight.setDisplayed(show);
  memory_weight.setDisplayed(show);
  sample_fraction.setDisplayed(show);
}

FlannMatcher::CompositeIndexParams::CompositeIndexParams(
		sofa::core::objectmodel::BaseObject* c)
    : trees(c->initData(&trees, 4, "COMPOSITETrees", "")),
      branching(c->initData(&branching, 32, "COMPOSITEBranching", "")),
      iterations(c->initData(&iterations, 11, "COMPOSITEIterations", "")),
      centers_init(c->initData(&centers_init, "COMPOSITECentersInit",
                               "default value: FLANN_CENTERS_RANDOM")),
      cb_index(c->initData(&cb_index, 0.2f, "COMPOSITECBIndex", ""))
{
  sofa::helper::OptionsGroup* t = centers_init.beginEdit();
  t->setNames(3, "FLANN_CENTERS_RANDOM", "FLANN_CENTERS_GONZALES",
              "FLANN_CENTERS_KMEANSPP", "FLANN_CENTERS_GROUPWISE");
  t->setSelectedItem("FLANN_CENTERS_RANDOM");
  centers_init.endEdit();
}

cv::Ptr<cv::flann::IndexParams>
FlannMatcher::CompositeIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::CompositeIndexParams>(
      trees.getValue(), branching.getValue(), iterations.getValue(),
      (cvflann::flann_centers_init_t)(centers_init.getValue().getSelectedId()),
      cb_index.getValue());
}

void FlannMatcher::CompositeIndexParams::toggleVisible(bool show)
{
  trees.setDisplayed(show);
  branching.setDisplayed(show);
  iterations.setDisplayed(show);
  centers_init.setDisplayed(show);
  cb_index.setDisplayed(show);
}

FlannMatcher::HierarchicalClusteringIndexParams::
		HierarchicalClusteringIndexParams(sofa::core::objectmodel::BaseObject* c)
    : branching(c->initData(&branching, 32, "HIERARCHICALBranching", "")),
      centers_init(c->initData(&centers_init, "HIERARCHICALCentersInit",
                               "default value: FLANN_CENTERS_RANDOM")),
      trees(c->initData(&trees, 4, "HIERARCHICALTrees", "")),
      leaf_size(c->initData(&leaf_size, 100, "HIERARCHICALLeafSize", ""))

{
  sofa::helper::OptionsGroup* t = centers_init.beginEdit();
  t->setNames(3, "FLANN_CENTERS_RANDOM", "FLANN_CENTERS_GONZALES",
              "FLANN_CENTERS_KMEANSPP", "FLANN_CENTERS_GROUPWISE");
  t->setSelectedItem("FLANN_CENTERS_RANDOM");
  centers_init.endEdit();
}

cv::Ptr<cv::flann::IndexParams>
FlannMatcher::HierarchicalClusteringIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::HierarchicalClusteringIndexParams>(
      branching.getValue(),
      (cvflann::flann_centers_init_t)(centers_init.getValue().getSelectedId()),
      trees.getValue(), leaf_size.getValue());
}

void FlannMatcher::HierarchicalClusteringIndexParams::toggleVisible(bool show)
{
  branching.setDisplayed(show);
  centers_init.setDisplayed(show);
  trees.setDisplayed(show);
  leaf_size.setDisplayed(show);
}

FlannMatcher::KDTreeIndexParams::KDTreeIndexParams(
		sofa::core::objectmodel::BaseObject* c)
    : trees(c->initData(&trees, 4, "KDTREETrees", ""))

{
}

cv::Ptr<cv::flann::IndexParams>
FlannMatcher::KDTreeIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::KDTreeIndexParams>(trees.getValue());
}

void FlannMatcher::KDTreeIndexParams::toggleVisible(bool show)
{
  trees.setDisplayed(show);
}

FlannMatcher::KMeansIndexParams::KMeansIndexParams(
		sofa::core::objectmodel::BaseObject* c)
    : branching(c->initData(&branching, 32, "KMEANSBranching", "")),
      iterations(c->initData(&iterations, 11, "KMEANSIterations", "")),
      centers_init(c->initData(&centers_init, "KMEANSCentersInit",
                               "default value: FLANN_CENTERS_RANDOM")),
      cb_index(c->initData(&cb_index, 0.2f, "KMEANSCBIndex", ""))
{
  sofa::helper::OptionsGroup* t = centers_init.beginEdit();
  t->setNames(3, "FLANN_CENTERS_RANDOM", "FLANN_CENTERS_GONZALES",
              "FLANN_CENTERS_KMEANSPP", "FLANN_CENTERS_GROUPWISE");
  t->setSelectedItem("FLANN_CENTERS_RANDOM");
  centers_init.endEdit();
}

cv::Ptr<cv::flann::IndexParams>
FlannMatcher::KMeansIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::KMeansIndexParams>(
      branching.getValue(), iterations.getValue(),
      (cvflann::flann_centers_init_t)(centers_init.getValue().getSelectedId()),
      cb_index.getValue());
}

void FlannMatcher::KMeansIndexParams::toggleVisible(bool show)
{
  branching.setDisplayed(show);
  iterations.setDisplayed(show);
  centers_init.setDisplayed(show);
  cb_index.setDisplayed(show);
}

FlannMatcher::LinearIndexParams::LinearIndexParams(
		sofa::core::objectmodel::BaseObject*)
{
}

cv::Ptr<cv::flann::IndexParams>
FlannMatcher::LinearIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::LinearIndexParams>();
}

void FlannMatcher::LinearIndexParams::toggleVisible(bool) {}
FlannMatcher::LshIndexParams::LshIndexParams(sofa::core::objectmodel::BaseObject* c)
    : table_number(c->initData(&table_number, 0, "LSHTableNumber", "")),
      key_size(c->initData(&key_size, 0, "LSHKeySize", "")),
      multi_probe_level(
          c->initData(&multi_probe_level, 0, "LSHMultiProbeLevel", ""))
{
}

cv::Ptr<cv::flann::IndexParams> FlannMatcher::LshIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::LshIndexParams>(table_number.getValue(),
                                                key_size.getValue(),
                                                multi_probe_level.getValue());
}

void FlannMatcher::LshIndexParams::toggleVisible(bool show)
{
  table_number.setDisplayed(show);
  key_size.setDisplayed(show);
  multi_probe_level.setDisplayed(show);
}

FlannMatcher::SavedIndexParams::SavedIndexParams(
		sofa::core::objectmodel::BaseObject* c)
    : filename(c->initData(&filename, std::string("SavedIndexParams"),
                           "SAVEDFilename", ""))
{
}

cv::Ptr<cv::flann::IndexParams> FlannMatcher::SavedIndexParams::getIndexParams()
{
  return cv::makePtr<cv::flann::SavedIndexParams>(filename.getValue());
}

void FlannMatcher::SavedIndexParams::toggleVisible(bool show)
{
  filename.setDisplayed(show);
}

FlannMatcher::SearchParams::SearchParams(sofa::core::objectmodel::BaseObject* c)
    : checks(c->initData(&checks, 32, "checks", "")),
      epsilon(c->initData(&epsilon, .0f, "epsilon", "")),
      sorted(c->initData(&sorted, true, "sorted", ""))
{
}

cv::Ptr<cv::flann::SearchParams> FlannMatcher::SearchParams::getSearchParams()
{
  return cv::makePtr<cv::flann::SearchParams>(
      checks.getValue(), epsilon.getValue(), sorted.getValue());
}

void FlannMatcher::SearchParams::toggleVisible(bool show)
{
  checks.setDisplayed(show);
  epsilon.setDisplayed(show);
  sorted.setDisplayed(show);
}

}  // namespace features
}  // namespace processor
}  // namespace sofaor
