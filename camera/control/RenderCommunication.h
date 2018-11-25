#ifndef SOFA_OR_PROCESSOR_RENDERCOMMUNICATION_H
#define SOFA_OR_PROCESSOR_RENDERCOMMUNICATION_H

#include "ImageProcessingPlugin.h"

#include "camera/common/CameraSettings.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include "serialization.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <SofaCV/SofaCV.h>
#include <sofa/helper/OptionsGroup.h>

#include <zmq.hpp>

namespace sofacv
{
namespace processor
{
class RenderCommunication : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      RenderCommunication, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Matrix3 Matrix3;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  typedef typename sofa::defaulttype::Quat Quat;

 public:
  SOFA_CLASS(RenderCommunication, ImplicitDataEngine);

  RenderCommunication():
      viewportwidth(
          initData(&viewportwidth, "viewportwidth", "width of the viewport")),
      viewportheight(initData(&viewportheight, "viewportheight", "height of the viewport")),
      viewportx(initData(&viewportx, "viewportx", "x position of the viewport")),
      viewporty(initData(&viewporty, "viewporty", "y position of the viewport")),
      port(initData(&port, "port", "ip port"))
  {
  }

  ~RenderCommunication() {}
  virtual void init() override
  {
    m_socket =new zmq::socket_t(m_context, ZMQ_PUB);
    std::string sipp= "tcp://127.0.0.1:" + std::to_string(port.getValue());
    m_socket->bind(sipp);

    update();
  }

  std::string save( const cv::Mat & mat )
  {
      std::ostringstream oss;
      boost::archive::text_oarchive toa( oss );
      toa << mat;

      return oss.str();
  }

  virtual void Update() override
  {
    int wdth = viewportwidth.getValue();
    int hght = viewportheight.getValue();
    int x = viewportx.getValue();
    int y = viewporty.getValue();

    GLfloat depths[hght * wdth ];
    GLfloat depthsN[hght * wdth ];
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    glEnable(GL_BLEND);
    //glEnable(GL_DEPTH_TEST);

    cv::Mat temp = cv::Mat::zeros(hght, wdth, CV_8UC1);
    cv::Mat tempI;

    glReadPixels(x, y, wdth, hght, GL_LUMINANCE, GL_UNSIGNED_BYTE,temp.data);


    // Process buffer so it matches correct format and orientation
    //cv::cvtColor(temp, tempI, CV_BGR2RGB);
    tempI = temp.clone();
    cv::flip(tempI, temp, 0);

    temp = (temp.reshape(0,1)); // to make it continuous
    // Write to file

    int  imgSize = temp.total()*temp.elemSize();

    std::string serialized = save(temp);

    // Send data here

     zmq::message_t message(serialized.length());
     memcpy(message.data(), serialized.c_str(), serialized.length());

     bool status = m_socket->send(message);
  }

  virtual void handleEvent(sofa::core::objectmodel::Event* e) override
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
      update();
    }
  }

  zmq::context_t     m_context{1};
  zmq::socket_t      *m_socket;
  sofa::Data<int> viewportwidth;
  sofa::Data<int> viewportheight;
  sofa::Data<int> viewportx;
  sofa::Data<int> viewporty;
  sofa::Data<int> port;

};

SOFA_DECL_CLASS(RenderCommunication)

int RenderCommunicationClass =
    sofa::core::RegisterObject(
        "Component to rotate a camera following any trajectory on a sphere, "
        "around a point. Init sets the correct camera orientation if necessary")
        .add<RenderCommunication>();

}  // namespace processor
}  // namespace sofaor
#endif  // SOFA_OR_PROCESSOR_ROTATEAROUNDENGINE_H
