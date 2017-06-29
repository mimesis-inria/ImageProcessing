#ifndef SOFA_OR_PROCESSOR_DATASLIDERMANAGER_INL
#define SOFA_OR_PROCESSOR_DATASLIDERMANAGER_INL

#include "DataSliderMgr.h"
#include "ImageFilter.h"

#include <sofa/helper/OptionsGroup.h>
#include <opencv2/highgui.hpp>

namespace sofa
{
namespace OR
{
namespace processor
{
// Default template methods for scalar data types and template specializations for Vec<N, U>

template <class T, class U>
void DataSliderManager<T, U>::createSlider(const std::string& winName)
{
	int value = getTrackbarRangedValue();
	cv::createTrackbar(m_data->getName(), winName, &value, getTrackbarMaxValue(),
										 &DataSliderManager<T, U>::callback, this);
	cv::setTrackbarPos(m_data->getName(), winName, value);
}

void DataSliderManager<helper::OptionsGroup, int>::createSlider(const std::string& winName)
{
	int value = getTrackbarRangedValue();
	cv::createTrackbar(m_data->getName(), winName, &value, getTrackbarMaxValue(),
										 &DataSliderManager<helper::OptionsGroup, int>::callback, this);
	cv::setTrackbarPos(m_data->getName(), winName, value);
}

void DataSliderManager<bool, bool>::createSlider(const std::string& winName)
{
	int value = getTrackbarRangedValue();
	cv::createTrackbar(m_data->getName(), winName, &value, getTrackbarMaxValue(),
										 &DataSliderManager<bool, bool>::callback, this);
	cv::setTrackbarPos(m_data->getName(), winName, value);
}

template <unsigned int N, class U>
void NDataSliderManager<N, U>::createSlider(const std::string& winName)
{
	int value = getTrackbarRangedValueX();
	cv::createTrackbar(m_data->getName(), winName, &value, getTrackbarMaxValue(),
										 &NDataSliderManager<N, U>::callback_x, this);
	cv::setTrackbarPos(m_data->getName() + ".x", winName, value);

	value = getTrackbarRangedValueY();
	cv::createTrackbar(m_data->getName(), winName, &value, getTrackbarMaxValue(),
										 &NDataSliderManager<N, U>::callback_y, this);
	cv::setTrackbarPos(m_data->getName() + ".y", winName, value);
	if (N > 2)
	{
		value = getTrackbarRangedValueZ();
		cv::createTrackbar(m_data->getName(), winName, &value,
											 getTrackbarMaxValue(),
											 &NDataSliderManager<N, U>::callback_z, this);
		cv::setTrackbarPos(m_data->getName() + ".z", winName, value);
	}
	if (N > 3)
	{
		value = getTrackbarRangedValueW();
		cv::createTrackbar(m_data->getName(), winName, &value,
											 getTrackbarMaxValue(),
											 &NDataSliderManager<N, U>::callback_w, this);
		cv::setTrackbarPos(m_data->getName() + ".w", winName, value);
	}
}




template <class T, class U>
int DataSliderManager<T, U>::getTrackbarMaxValue()
{
	return int((m_max - m_min) / m_step);
}

int DataSliderManager<helper::OptionsGroup, int>::getTrackbarMaxValue()
{
	return int((m_max - m_min) / m_step);
}

int DataSliderManager<bool, bool>::getTrackbarMaxValue() { return 1; }

template <unsigned int N, class U>
int NDataSliderManager<N, U>::getTrackbarMaxValue()
{
	return int((m_max - m_min) / m_step);
}




template <class T, class U>
int DataSliderManager<T, U>::getTrackbarRangedValue()
{
	return int((m_data->getValue() - m_min) / m_step);
}

int DataSliderManager<helper::OptionsGroup, int>::getTrackbarRangedValue()
{
	return int(m_data->getValue().getSelectedId());
}

int DataSliderManager<bool, bool>::getTrackbarRangedValue()
{
	return int(m_data->getValue());
}

template <unsigned int N, class U>
int NDataSliderManager<N, U>::getTrackbarRangedValueX()
{
	assert(N >= 2);
	return int((m_data->getValue().x() - m_min) / m_step);
}

template <unsigned int N, class U>
int NDataSliderManager<N, U>::getTrackbarRangedValueY()
{
	assert(N >= 2);
	return int((m_data->getValue().y() - m_min) / m_step);
}

template <unsigned int N, class U>
int NDataSliderManager<N, U>::getTrackbarRangedValueZ()
{
	assert(N >= 3);
	return int((reinterpret_cast<NDataSliderManager<3, U>*>(this)
									->m_data->getValue()
									.z() -
							m_min) /
						 m_step);
}

template <unsigned int N, class U>
int NDataSliderManager<N, U>::getTrackbarRangedValueW()
{
	assert(N >= 4);
	return int((reinterpret_cast<NDataSliderManager<4, U>*>(this)
									->m_data->getValue()
									.w() -
							m_min) /
						 m_step);
}





template <class T, class U>
void DataSliderManager<T, U>::callback(int val, void* mgr)
{
	DataSliderManager<T, U>* m = reinterpret_cast<DataSliderManager<T, U>*>(mgr);
	if (m->getTrackbarRangedValue() != val)
	{
		m->m_data->setValue(U(val) * m->m_step + m->m_min);
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}

void DataSliderManager<helper::OptionsGroup, int>::callback(int val, void* mgr)
{
	DataSliderManager<helper::OptionsGroup, int>* m =
			reinterpret_cast<DataSliderManager<helper::OptionsGroup, int>*>(mgr);
	if (m->getTrackbarRangedValue() != val)
	{
		m->m_data->beginEdit()->setSelectedItem(unsigned(val));
		m->m_data->endEdit();
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}

void DataSliderManager<bool, bool>::callback(int val, void* mgr)
{
	DataSliderManager<bool, bool>* m =
			reinterpret_cast<DataSliderManager<bool, bool>*>(mgr);
	if (m->getTrackbarRangedValue() != val)
	{
		m->m_data->setValue(bool(val));
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}

template <unsigned int N, class U>
void NDataSliderManager<N, U>::callback_x(int val, void* mgr)
{
	assert(N >= 2);
	NDataSliderManager<2, U>* m =
			reinterpret_cast<NDataSliderManager<2, U>*>(mgr);
	if (m->getTrackbarRangedValueX() != val)
	{
		m->m_data->beginEdit()->x() = U(val) * m->m_step + m->m_min;
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}

template <unsigned int N, class U>
void NDataSliderManager<N, U>::callback_y(int val, void* mgr)
{
	assert(N >= 2);
	NDataSliderManager<2, U>* m =
			reinterpret_cast<NDataSliderManager<2, U>*>(mgr);
	if (m->getTrackbarRangedValueY() != val)
	{
		m->m_data->beginEdit()->y() = U(val) * m->m_step + m->m_min;
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}

template <unsigned int N, class U>
void NDataSliderManager<N, U>::callback_z(int val, void* mgr)
{
	assert(N >= 3);

	NDataSliderManager<3, U>* m =
			reinterpret_cast<NDataSliderManager<3, U>*>(mgr);
	if (m->getTrackbarRangedValueZ() != val)
	{
		m->m_data->beginEdit()->z() = U(val) * m->m_step + m->m_min;
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}

template <unsigned int N, class U>
void NDataSliderManager<N, U>::callback_w(int val, void* mgr)
{
	assert(N >= 4);
	NDataSliderManager<4, U>* m =
			reinterpret_cast<NDataSliderManager<4, U>*>(mgr);
	if (m->getTrackbarRangedValueW() != val)
	{
		m->m_data->beginEdit()->w() = U(val) * m->m_step + m->m_min;
		dynamic_cast<ImageFilter*>(m->m_data->getOwner())->refreshDebugWindow();
	}
}


}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_DATASLIDERMANAGER_INL
