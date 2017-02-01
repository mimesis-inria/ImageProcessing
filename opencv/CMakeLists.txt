cmake_minimum_required(VERSION 2.8.12)
project(SofaOROpenCV)

set(SOFAOROPENCV_MAJOR_VERSION 0)
set(SOFAOROPENCV_MINOR_VERSION 1)
set(SOFAOROPENCV_VERSION ${SOFAOROPENCV_MAJOR_VERSION}.${SOFAOROPENCV_MINOR_VERSION})

set(HEADER_FILES
  initPlugin.h
  core/ImageFilter.h
  features/Detectors.h
  features/Matchers.h
  features/FeatureDetector.h
  features/DescriptorMatcher.h
  features/FeatureTriangulator.h
  features/MatchingConstraints.h
  features/OpticalFlow.h
  imgproc/CannyFilter.h
  imgproc/ImageRectifier.h
  #imgproc/ImageConverter.h
  calib/CalibLoader.h
  )

set(SOURCE_FILES
  initPlugin.cpp
  core/ImageFilter.cpp
  features/Detectors.cpp
  features/Matchers.cpp
  features/FeatureDetector.cpp
  features/DescriptorMatcher.cpp
  features/FeatureTriangulator.cpp
  features/MatchingConstraints.cpp
  features/OpticalFlow.cpp
  imgproc/CannyFilter.cpp
  imgproc/ImageRectifier.cpp
  #imgproc/ImageConverter.cpp
  calib/CalibLoader.cpp
  )

set(OpenCV_LIBRARIES SofaCore SofaSimulationCommon SofaORCommon ${OpenCV_LIBRARIES})

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/.>")

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${SOFAOROPENCV_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

add_definitions("-DSOFA_BUILD_SOFAOROPENCV")

## Install rules for the library; CMake package configurations files
sofa_create_package(${PROJECT_NAME} ${SOFAOROPENCV_VERSION} ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/" DESTINATION "include/processor/opencv"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")
