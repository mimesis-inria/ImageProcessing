#ifndef SOFA_OR_PROCESSOR_IMAGEFILTER_H
#define SOFA_OR_PROCESSOR_IMAGEFILTER_H

#include <SofaORCommon/cvMat.h>
#include <sofa/core/DataEngine.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa
{
namespace OR
{
namespace processor
{
class ImageFilter : public core::DataEngine
{
  static void callback(int val, void* holder);

 public:
  SOFA_CLASS(ImageFilter, core::DataEngine);

  ImageFilter();
  virtual ~ImageFilter();

  virtual void init();
  virtual void update();
  virtual void reinit();

  // Implement the filter in this method;
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out, bool debug = false) = 0;

  // redraw the image for debugging purposes when changes are previewed on the
  // filter, but not yet applied
  virtual void drawDebug();
  void refreshDebugWindow();
  bool reinitDebugWindow();

  Data<common::cvMat> d_in;
  Data<common::cvMat> d_out;
  Data<bool> d_displayDebugWindow;

  virtual void handleEvent(sofa::core::objectmodel::Event* event)
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(event))
      this->update();
  }

  // Pass data to this methods to bind them to the OpenCV UI
  void registerData(Data<bool>* data, int min = 0, int max = 1, int step = 1);
  void registerData(Data<int>* data, int min, int max, int step);
  void registerData(Data<double>* data, double min, double max, double step);
  void registerData(Data<float>* data, float min, float max, float step);
  void registerData(Data<helper::OptionsGroup>* data, int min, int max, int step);
  void unregisterAllData();

 protected:
  static unsigned m_window_uid;
  cv::Mat m_debugImage;

  // if set to false, will not write in the output image (useful for filters
  // such as FeatureDetectors / matchers, where we want to visualize filters in
  // the debug window, but we don't need the debug output for other filters)
  bool m_outputImage;

 private:
  struct Holder
  {
    enum Type
    {
      BOOL,
      INT,
      DOUBLE,
      OPTIONSGROUP,
    } type;

    union Impl {
      bool _bool;
      int _int;
      double _double;
    };

    core::objectmodel::BaseData* data;

    Impl value_min, value_max, step;

    template <typename T>
    Holder(Type _type, core::objectmodel::BaseData* _data, T min, T max,
           T _step)
        : type(_type), data(_data)
    {
      switch (type)
      {
        case BOOL:
          value_min._int = 0;
          value_max._int = 1;
          step._int = 1;
        case INT:
        case OPTIONSGROUP:
          value_min._int = min;
          value_max._int = max;
          step._int = _step;
          break;
        case DOUBLE:
          value_min._double = min;
          value_max._double = max;
          step._double = _step;
          break;
      }
    }
    int getTrackbarRangedValue();
    int getTrackbarMaxValue();
    void setDataValue(int val);
    void refresh();
  };

  std::vector<Holder> m_params;
  const std::string m_win_name;
  void getInputFromContext();
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_IMAGEFILTER_H
