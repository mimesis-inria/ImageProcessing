#include "CannyFilter.h"

using namespace sofa::OR::processor;
using namespace sofa;

SOFA_DECL_CLASS(CannyFilter)

int CannyFilterClass =
    core::RegisterObject("Canny edge detection filter from OpenCV")
        .add<CannyFilter>();
