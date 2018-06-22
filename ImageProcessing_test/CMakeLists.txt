cmake_minimum_required(VERSION 3.1)

project(ImageProcessing_test)

find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

set(HEADER_FILES
 regression/Regression_test.h
)

set(SOURCE_FILES
 camera/common/CameraSettings_test.cpp
 common/DataSliderMgr_test.cpp
 regression/Regression_test.cpp
)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaTest SofaGTestMain SofaCV ImageProcessing ${OPENSSL_LIBRARIES})

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
