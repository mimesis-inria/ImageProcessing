#ifndef SOFA_OR_PROCESSOR_ScalarSliderManager_H
#define SOFA_OR_PROCESSOR_ScalarSliderManager_H

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
struct DSM
{
	virtual void createSlider(const std::string& winName) = 0;
};

template <class T, class U>
struct ScalarSliderManager : DSM
{
	ScalarSliderManager(sofa::Data<T>* d, U min, U max, U step)
	{
		this->m_data = d;
		this->m_min = min;
		this->m_max = max;
		this->m_step = step;
	}
	~ScalarSliderManager() {}

	sofa::Data<T>* m_data;
	U m_min, m_max, m_step;

	void createSlider(const std::string& winName);
	int getTrackbarMaxValue();
	int getTrackbarRangedValue();
	static void callback(int, void*);
};

template <>
struct ScalarSliderManager<helper::OptionsGroup, int> : DSM
{
	ScalarSliderManager(sofa::Data<helper::OptionsGroup>* d)
	{
		this->m_data = d;
		this->m_min = 0;
		this->m_max = d->getValue().size() - 1;
		this->m_step = 1;
	}
	~ScalarSliderManager() {}

	void createSlider(const std::string& winName);
	int getTrackbarMaxValue();
	int getTrackbarRangedValue();
	static void callback(int, void*);

	sofa::Data<helper::OptionsGroup>* m_data;
	int m_min, m_max, m_step;
};

template <>
struct ScalarSliderManager<bool, bool> : DSM
{
	ScalarSliderManager(sofa::Data<bool>* d)
	{
		this->m_data = d;
		this->m_min = 0;
		this->m_max = 1;
		this->m_step = 1;
	}

	~ScalarSliderManager() {}

	void createSlider(const std::string& winName);
	int getTrackbarMaxValue();
	int getTrackbarRangedValue();
	static void callback(int, void*);

	sofa::Data<bool>* m_data;
	bool m_min, m_max, m_step;
};

template <unsigned int N, class U>
struct VecSliderManager : DSM
{
	VecSliderManager(sofa::Data<defaulttype::Vec<N, U> >* d, U min, U max, U step)
	{
		this->m_data = d;
		this->m_min = min;
		this->m_max = max;
		this->m_step = step;
	}
	~VecSliderManager() {}

	void createSlider(const std::string& winName);

	int getTrackbarMaxValue();
	int getTrackbarRangedValueX();
	int getTrackbarRangedValueY();
	int getTrackbarRangedValueZ();
	int getTrackbarRangedValueW();

	static void callback_x(int val, void* mgr);
	static void callback_y(int val, void* mgr);
	static void callback_z(int val, void* mgr);
	static void callback_w(int val, void* mgr);

	sofa::Data<defaulttype::Vec<N, U> >* m_data;
	U m_min, m_max, m_step;
};

}  // namespace processor
}  // namespace OR
}  // namespace sofa

#endif  // SOFA_OR_PROCESSOR_ScalarSliderManager_H
