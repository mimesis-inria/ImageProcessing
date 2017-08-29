#ifndef SOFA_OR_PROCESSOR_IMAGEFILTER_NEWGUI_H
#define SOFA_OR_PROCESSOR_IMAGEFILTER_NEWGUI_H

#include <SofaORCommon/ImplicitDataEngine.h>
#include <SofaORCommon/cvMat.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofaor
{
namespace processor
{
/**
 *  \brief provides a Debug UI mechanism for image filters
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
 * void applyFilter(const cv::Mat& in, cv::Mat& out, bool **debug**)
 * {
 *   // code here your filter's behavior
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
 public:
  SOFA_CLASS(ImageFilter, common::ImplicitDataEngine);

  /**
   * @brief ImageFilter
   * @param outputImage set to false if you don't want this filter to output an
   * image useful for filters
   * such as FeatureDetectors / matchers, where we want to visualize the
   * resulting image in
   * the debug window, but we don't need the debug output for further
   * processing)
   */
  ImageFilter();
  virtual ~ImageFilter();

  virtual void init();
  virtual void update();
  virtual void reinit();
  void bindGlTexture(const std::string& imageString);
  virtual void drawFullFrame();

  /**
   * Implement the filter's behavior in this method;
   * @param[in] in input image matrix
   * @param[out] output image matrix to modify
   * @param debug true if applyFilter is called from the mouse callback or from
   * a modified slider in the Debug UI
   */
  virtual void applyFilter(const cv::Mat& /*in*/, cv::Mat& /*out*/,
                           bool /*debug*/ = false)
  {
  }

  /// Creates the Debug UI window if not created yet, sets its registered
  /// trackbars, and activates the necessary callbacks
  void refreshDebugWindow();

  sofa::Data<common::cvMat> d_img;      ///< [INPUT] image to process
  sofa::Data<common::cvMat> d_img_out;  ///< [OUTPUT] processed image
  sofa::Data<bool> d_isActive;  ///< Whether the filter is performed or not
  sofa::Data<bool> d_outputImage;

  /// activates the mouse callback when mouse interaction is performed on the
  /// image's debug UI
  void activateMouseCallback();
  /**
   *
   *
   * \fn void registerData(sofa::Data<bool>* data)
   *
   * \brief Pass data to @registerData to bind them to ImageFilter's debug UI
   *
   * @param[in] data the data to bind to a slider
   *
   */
  void registerData(sofa::Data<bool>* data);

  /// @see void registerData(sofa::Data<bool>* data)
  void registerData(sofa::Data<sofa::helper::OptionsGroup>* data);

  /**
   * \fn void registerData(sofa::Data<uchar>* data, uchar min, uchar max, uchar\
   * step)
   *
   * \brief Pass data to @registerData to bind them to ImageFilter's debug UI
   *
   * @param[in] data the data to bind to a slider
   * @param[in] min the minimum value of the data
   * @param[in] max the maximum value of the data
   * @param[in] step the discretized interval for the slider
   *
   */
  void registerData(sofa::Data<uchar>* data, uchar min, uchar max, uchar step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<int>* data, int min, int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<unsigned>* data, unsigned min, unsigned max,
                    unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<double>* data, double min, double max,
                    double step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<float>* data, float min, float max, float step);

  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2u>* data, unsigned min,
                    unsigned max, unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3u>* data, unsigned min,
                    unsigned max, unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4u>* data, unsigned min,
                    unsigned max, unsigned step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2i>* data, int min,
                    int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3i>* data, int min,
                    int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4i>* data, int min,
                    int max, int step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2f>* data, float min,
                    float max, float step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3f>* data, float min,
                    float max, float step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4f>* data, float min,
                    float max, float step);

  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec2d>* data, double min,
                    double max, double step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec3d>* data, double min,
                    double max, double step);
  /// @see void registerData(sofa::Data<uchar>* data, uchar min, uchar max,
  /// uchar step)
  void registerData(sofa::Data<sofa::defaulttype::Vec4d>* data, double min,
                    double max, double step);

  /// unregisters all data passed through @see registerData() for the Debug UI
  void unregisterAllData();

 protected:
  /// mouse callback (if activated through @see activateMouseCallback()
  virtual void mouseCallback(int, int, int, int) {}

 private:
  static unsigned m_window_uid;
  bool m_isMouseCallbackActive;
  cv::Mat m_debugImage;
  void reinitDebugWindow();
};

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_IMAGEFILTER_NEWGUI_H
