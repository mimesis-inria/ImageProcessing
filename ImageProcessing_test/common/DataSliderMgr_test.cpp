#include "DataSliderMgr.h"
#include <SofaTest/Sofa_test.h>
#include <opencv2/opencv.hpp>


namespace sofa
{
struct DSM_test : public sofa::Sofa_test<>
{
//	ScalarSliderManager<bool, bool>* boolSlider;
//	ScalarSliderManager<helper::OptionsGroup, int>* boolSlider;
//	ScalarSliderManager<int, int>* boolSlider;
//	ScalarSliderManager<double, double>* boolSlider;

	DSM_test()
	{

	}


	void createBoolSlider()
	{

	}

	void SetUp()
	{
		cv::namedWindow("window");
	}

	void TearDown()
	{
		cv::destroyAllWindows();
	}
};

TEST_F(DSM_test, createBoolSlider) { this->createBoolSlider(); }

}  // namespace sofa
