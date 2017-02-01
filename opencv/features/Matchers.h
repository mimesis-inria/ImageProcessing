#ifndef SOFA_OR_PROCESSOR_MATCHERS_H
#define SOFA_OR_PROCESSOR_MATCHERS_H

#include <SofaORCommon/cvMat.h>

#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vector.h>

#include <opencv2/xfeatures2d.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
struct BaseMatcher
{
  virtual ~BaseMatcher();

  virtual void toggleVisible(bool) = 0;

  virtual void init() = 0;
  virtual bool acceptsBinary() = 0;

  virtual void knnMatch(const common::cvMat& queryDescriptors,
                        const common::cvMat& trainDescriptors,
                        std::vector<std::vector<cv::DMatch> >& matches, int k,
                        const common::cvMat& mask);
  virtual void radiusMatch(const common::cvMat& queryDescriptors,
                           const common::cvMat& trainDescriptors,
                           std::vector<std::vector<cv::DMatch> >& matches,
                           float maxDistance, const common::cvMat& mask);

 protected:
  cv::DescriptorMatcher* m_matcher;
};

struct BFMatcher : BaseMatcher
{
  static const int cvNorms[4];

  BFMatcher(core::objectmodel::BaseObject* c);
  void toggleVisible(bool);
  void init();
  bool acceptsBinary() { return true; }
  Data<sofa::helper::OptionsGroup> normType;
  Data<bool> crossCheck;
};

struct FlannMatcher : BaseMatcher
{
  FlannMatcher(core::objectmodel::BaseObject* c);
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
    AutotunedIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<float> target_precision;
    Data<float> build_weight;
    Data<float> memory_weight;
    Data<float> sample_fraction;
  };

  struct CompositeIndexParams : IndexParams
  {
    CompositeIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<int> trees;
    Data<int> branching;
    Data<int> iterations;
    Data<helper::OptionsGroup> centers_init;
    Data<float> cb_index;
  };
  struct HierarchicalClusteringIndexParams : IndexParams
  {
    HierarchicalClusteringIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<int> branching;
    Data<helper::OptionsGroup> centers_init;
    Data<int> trees;
    Data<int> leaf_size;
  };
  struct KDTreeIndexParams : IndexParams
  {
    KDTreeIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<int> trees;
  };
  struct KMeansIndexParams : IndexParams
  {
    KMeansIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<int> branching;
    Data<int> iterations;
    Data<helper::OptionsGroup> centers_init;
    Data<float> cb_index;
  };
  struct LinearIndexParams : IndexParams
  {
    LinearIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();
  };
  struct LshIndexParams : IndexParams
  {
    LshIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<int> table_number;
    Data<int> key_size;
    Data<int> multi_probe_level;
  };
  struct SavedIndexParams : IndexParams
  {
    SavedIndexParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::IndexParams> getIndexParams();

    Data<std::string> filename;
  };

  struct SearchParams
  {
    SearchParams(core::objectmodel::BaseObject* c);
    void toggleVisible(bool);
    cv::Ptr<cv::flann::SearchParams> getSearchParams();

    Data<int> checks;
    Data<float> epsilon;
    Data<bool> sorted;
  };

 public:
  Data<sofa::helper::OptionsGroup> indexParamsType;
  IndexParams* m_AllIndexParams[8];
  SearchParams* m_searchParams;
  IndexParams* m_indexParams;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_MATCHERS_H
