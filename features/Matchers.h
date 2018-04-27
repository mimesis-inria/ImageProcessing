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

#ifndef SOFACV_FEATURES_MATCHERS_H
#define SOFACV_FEATURES_MATCHERS_H

#include <SofaCV/SofaCV.h>

#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vector.h>

#include <opencv2/xfeatures2d.hpp>

namespace sofacv
{
namespace features
{
struct BaseMatcher
{
  virtual ~BaseMatcher();

  virtual void toggleVisible(bool) = 0;

  virtual void init() = 0;
  virtual bool acceptsBinary() = 0;

  virtual void knnMatch(const cvMat& queryDescriptors,
                        const cvMat& trainDescriptors,
                        std::vector<std::vector<cv::DMatch> >& matches, int k,
                        const cvMat& mask);
  virtual void radiusMatch(const cvMat& queryDescriptors,
                           const cvMat& trainDescriptors,
                           std::vector<std::vector<cv::DMatch> >& matches,
                           float maxDistance, const cvMat& mask);

 protected:
  cv::DescriptorMatcher* m_matcher;
};

struct BFMatcher : BaseMatcher
{
  static const int cvNorms[4];

  BFMatcher(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  bool acceptsBinary() { return true; }
  sofa::Data<sofa::helper::OptionsGroup> normType;
  sofa::Data<bool> crossCheck;
};

struct FlannMatcher : BaseMatcher
{
  FlannMatcher(sofa::core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  bool acceptsBinary()
  {
    return (dynamic_cast<LshIndexParams*>(m_indexParams)) ? (true) : (false);
  }

 public:
  enum IndexParamsType
  {
    AUTOTUNED = 0,
    COMPOSITE = 1,
    HIERARCHICAL_CLUSTERING = 2,
    KDTREE = 3,
    KMEANS = 4,
    LINEAR = 5,
    LSH = 6,
    SAVED = 7,
    IndexParamsType_COUNT
  };

  struct IndexParams
  {
    virtual ~IndexParams() {}
    virtual void toggleVisible(bool) = 0;
    virtual cv::Ptr<cv::flann::IndexParams> getIndexParams() = 0;
  };

  struct AutotunedIndexParams : IndexParams
  {
    AutotunedIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<float> target_precision;
    sofa::Data<float> build_weight;
    sofa::Data<float> memory_weight;
    sofa::Data<float> sample_fraction;
  };

  struct CompositeIndexParams : IndexParams
  {
    CompositeIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<int> trees;
    sofa::Data<int> branching;
    sofa::Data<int> iterations;
    sofa::Data<sofa::helper::OptionsGroup> centers_init;
    sofa::Data<float> cb_index;
  };
  struct HierarchicalClusteringIndexParams : IndexParams
  {
    HierarchicalClusteringIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<int> branching;
    sofa::Data<sofa::helper::OptionsGroup> centers_init;
    sofa::Data<int> trees;
    sofa::Data<int> leaf_size;
  };
  struct KDTreeIndexParams : IndexParams
  {
    KDTreeIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<int> trees;
  };
  struct KMeansIndexParams : IndexParams
  {
    KMeansIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<int> branching;
    sofa::Data<int> iterations;
    sofa::Data<sofa::helper::OptionsGroup> centers_init;
    sofa::Data<float> cb_index;
  };
  struct LinearIndexParams : IndexParams
  {
    LinearIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();
  };
  struct LshIndexParams : IndexParams
  {
    LshIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<int> table_number;
    sofa::Data<int> key_size;
    sofa::Data<int> multi_probe_level;
  };
  struct SavedIndexParams : IndexParams
  {
    SavedIndexParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    sofa::Data<std::string> filename;
  };

  struct SearchParams
  {
    SearchParams(sofa::core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::SearchParams> getSearchParams();

    sofa::Data<int> checks;
    sofa::Data<float> epsilon;
    sofa::Data<bool> sorted;
  };

 public:
  sofa::Data<sofa::helper::OptionsGroup> indexParamsType;
  IndexParams* m_AllIndexParams[8];
  SearchParams* m_searchParams;
  IndexParams* m_indexParams;
};

}  // namespace features
}  // namespace sofacv

#endif  // SOFACV_FEATURES_MATCHERS_H
