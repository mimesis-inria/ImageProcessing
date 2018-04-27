cmake_minimum_required(VERSION 3.1)

project(ImageProcessing_test)

set(HEADER_FILES
)

set(SOURCE_FILES
  camera/common/CameraSettings_test.cpp
  common/DataSliderMgr_test.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} SofaTest SofaGTestMain SofaCV ImageProcessing)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
