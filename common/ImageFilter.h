#ifndef SOFA_OR_PROCESSOR_IMAGEFILTER_H
#define SOFA_OR_PROCESSOR_IMAGEFILTER_H

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMat.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa
{
namespace OR
{
namespace processor
{
/**
 *  \brief provides UI mechanisms for image filters
 *
 * Implementation good rules: @see ImplicitDataEngine for additional impl rules
 *
 * void init()
 * {
 *    // if your filter needs parameter tuning
 *    registerData(&d_data, minVal, maxVal, step)
 *
 *    // if your filter requires manual interaction (e.g. Segmenter2D)
 *    activateMouseCallback();
 *
 *    ImageFilter::init(); // Always call at the end of init()
 * }
 *
 * // optional
 * void reinit()
 * {
 *   ImageFilter::reinit();
 * }
 *
 * void update()
 * {
 *   ImageFilter::update();
 * }
 *
 */
class ImageFilter : public common::ImplicitDataEngine
{
  static void callback(int val, void* holder);

  static void _mouseCallback(int e, int x, int y, int f, void* d)
  {
    reinterpret_cast<ImageFilter*>(d)->mouseCallback(e, x, y, f);
  }

 public:
  SOFA_CLASS(ImageFilter, common::ImplicitDataEngine);

  ImageFilter(bool outputImage = true);
  virtual ~ImageFilter();

  virtual void init();
  virtual void update();
  virtual void reinit();

  // Implement the filter in this method;
  virtual void applyFilter(const cv::Mat& in, cv::Mat& out,
                           bool debug = false) = 0;

  // Creates the debugging window and its associated trackbars
  void reinitDebugWindow();
	void refreshDebugWindow();

  Data<common::cvMat> d_img;
  Data<common::cvMat> d_img_out;
  Data<bool> d_displayDebugWindow;
  Data<bool> d_isActive;

  void activateMouseCallback();
  // Pass data to this methods to bind them to the OpenCV UI
  void registerData(Data<bool>* data);
  void registerData(Data<int>* data, int min, int max, int step);
  void registerData(Data<double>* data, double min, double max, double step);
  void registerData(Data<float>* data, float min, float max, float step);
  void registerData(Data<helper::OptionsGroup>* data);
  void unregisterAllData();

	void drawImage();
 protected:
  virtual void mouseCallback(int, int, int, int) {}
  static unsigned m_window_uid;
  cv::Mat m_debugImage;

  // if set to false, will not write in the output image (useful for filters
  // such as FeatureDetectors / matchers, where we want to visualize filters in
  // the debug window, but we don't need the debug output for other filters)
  bool m_outputImage;
  bool m_isMouseCallbackActive;

private:
	core::DataTracker m_displayDebugDataTracker;
  struct Holder
  {
    enum Type
    {
      BOOL,
      INT,
      DOUBLE,
      FLOAT,
      OPTIONSGROUP,
    } type;

    union Impl {
      bool _bool;
      int _int;
      double _double;
      float _float;
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
        case FLOAT:
          value_min._float = min;
          value_max._float = max;
          step._float = _step;
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
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_IMAGEFILTER_H