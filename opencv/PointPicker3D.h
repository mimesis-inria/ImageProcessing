#ifndef SOFA_OR_PROCESSOR_POINTPICKER3D_H
#define SOFA_OR_PROCESSOR_POINTPICKER3D_H

#include "initPlugin.h"

#include <SofaORCommon/ImplicitDataEngine.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/gui/PickHandler.h>
#include "calib/CameraSettings.h"

namespace sofa
{
namespace OR
{
namespace processor
{
// TODO: move to helper
// https://www.opengl.org/wiki/GluProject_and_gluUnProject_code
template <class Real>
bool glhUnProjectf(Real winx, Real winy, Real winz, Real* modelview,
									 Real* projection,
									 const core::visual::VisualParams::Viewport& viewport,
									 Real* objectCoordinate)
{
	// Transformation matrices
	sofa::defaulttype::Mat<4, 4, Real> matModelview(modelview);
	sofa::defaulttype::Mat<4, 4, Real> matProjection(projection);

	sofa::defaulttype::Mat<4, 4, Real> m, A;
	sofa::defaulttype::Vec<4, Real> in, out;

	A = matProjection * matModelview;
	sofa::defaulttype::invertMatrix(m, A);

	// Transformation of normalized coordinates between -1 and 1
	in[0] = (winx - (Real)viewport[0]) / (Real)viewport[2] * 2.0 - 1.0;
	in[1] = (winy - (Real)viewport[1]) / (Real)viewport[3] * 2.0 - 1.0;
	in[2] = 2.0 * winz - 1.0;
	in[3] = 1.0;
	// Objects coordinates
	out = m * in;

	if (out[3] == 0.0) return false;
	out[3] = 1.0 / out[3];
	objectCoordinate[0] = out[0] * out[3];
	objectCoordinate[1] = out[1] * out[3];
	objectCoordinate[2] = out[2] * out[3];
	return true;
}

class PointPicker3D : public common::ImplicitDataEngine
{
	typedef sofa::core::objectmodel::SingleLink<CalibratedCamera, CameraSettings,
																							BaseLink::FLAG_STOREPATH |
																									BaseLink::FLAG_STRONGLINK>
			CamSettings;

 public:
	PointPicker3D() : isActive(false)
	{
		pick = new PickHandler();
		pick->setPickingMethod(gui::PickHandler::RAY_CASTING);
		pick->init(groot.get());
	}

	void activatePicker(int x, int y)
	{
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		pick->activateRay(viewport[2], viewport[3], getContext()->getRootContext());

		defaulttype::Vector3 origin, direction;

		{
			const sofa::core::visual::VisualParams* vp =
					sofa::core::visual::VisualParams::defaultInstance();

			double winX = (double)x;
			double winY = (double)viewport[3] - (double)y;

			double pos[3];
//			// From CameraSettings
//      double modelview[16] = l_cam->getGLModelview().ptr();
//      double projection[16] = l_cam->getGLProjection().ptr();
			// From OpenGL
			double modelview[16];
			double projection[16];
			glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
			glGetDoublev(GL_PROJECTION_MATRIX, projection);

			float fwinZ = 0.0;
			vp->drawTool()->readPixels(x, int(winY), 1, 1, NULL, &fwinZ);

			double winZ = (double)fwinZ;
			glhUnProjectf<double>(winX, winY, winZ, modelview, projection, viewport,
														pos);

			return Vec3(pos[0], pos[1], pos[2]);
		}

		pick->findCollisionUsingBruteForce(origin, direction, 1000.0,
																			 getContext()->getRootContext());
	}

	virtual void handleEvent(sofa::core::objectmodel::Event* e)
	{
		if (sofa::core::objectmodel::MouseEvent::checkEventType(e))
		{
			sofa::core::objectmodel::MouseEvent* me =
					static_cast<core::objectmodel::MouseEvent*>(e);
			int posX = me->getPosX();
			int posY = me->getPosY();
			if (me->getState() == core::objectmodel::MouseEvent::LeftPressed)
			{
				activatePicker(posX, posY);
			}
		}
		ImplicitDataEngine::handleEvent(e);
	}

 private:
	CamSettings l_cam;
	bool isActive;
	gui::PickHandler* pick;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif SOFA_OR_PROCESSOR_POINTPICKER3D_H
