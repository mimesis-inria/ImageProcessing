#ifndef SOFA_OR_PROCESSOR_IMAGEFILTER_H
#define SOFA_OR_PROCESSOR_IMAGEFILTER_H

#include <SofaORCommon/cvMat.h>
#include <sofa/core/DataEngine.h>
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
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out) = 0;

  // redraw the image for debugging purposes when changes are previewed on the
  // filter, but not yet applied
  virtual void drawDebug();
  void refreshDebugWindow();
  void reinitDebugWindow();

  Data<common::cvMat> d_in;
  Data<common::cvMat> d_out;
  Data<bool> d_displayDebugWindow;
  bool m_debugDisplayed;

  virtual void handleEvent(sofa::core::objectmodel::Event* event)
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(event))
      this->update();
  }

  // Pass data to this methods to bind them to the OpenCV UI
  void registerData(Data<bool>* data);
  void registerData(Data<int>* data, int min, int max, int step);
  void registerData(Data<double>* data, double min, double max, double step);

 protected:
  static unsigned m_window_uid;
  cv::Mat m_debugImage;

 private:
  struct Holder
  {
    enum Type
    {
      BOOL,
      INT,
      DOUBLE,
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

    Holder(core::objectmodel::BaseData* data) : type(BOOL), data(data) {}
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
