cmake_minimum_required(VERSION 2.8.12)
project(ProcessOR)

set(PROCESSOR_MAJOR_VERSION 0)
set(PROCESSOR_MINOR_VERSION 1)
set(PROCESSOR_VERSION ${PROCESSOR_MAJOR_VERSION}.${PROCESSOR_MINOR_VERSION})

set(HEADER_FILES
  initplugin.h
  Detectors.h
  Matchers.h
  FeatureDetector.h
  DescriptorMatcher.h
  FeatureTriangulator.h
  MatchingConstraints.h
  ImageFilter.h
  CannyFilter.h
  ImageConverter.h
  ImageRectifier.h
  FeatureRectifier.h
  OpticalFlow.h
  )

set(SOURCE_FILES
  initPlugin.cpp
  Detectors.cpp
  Matchers.cpp
  FeatureDetector.cpp
  DescriptorMatcher.cpp
  FeatureTriangulator.cpp
  MatchingConstraints.cpp
  ImageFilter.cpp
  CannyFilter.cpp
  ImageConverter.cpp
  ImageRectifier.cpp
  FeatureRectifier.cpp
  OpticalFlow.cpp
  )

set(ProcessOR_LIBRARIES SofaCore SofaSimulationCommon AcquisitOR SofaORCommon ${OpenCV_LIBRARIES})

if (SOFAOR_ENABLE_PCL)
  add_definitions("-DSOFAOR_USE_PCL")
  find_package(PCL COMPONENTS common octree io kdtree search sample_consensus filters features segmentation surface  REQUIRED )

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})

  list( APPEND SOURCE_FILES
    pcl/PCSmootherMLS.cpp
    pcl/PCDownsampler.cpp)
  list( APPEND HEADER_FILES
    pcl/PCSmootherMLS.h
    pcl/PCDownsampler.h)
endif(SOFAOR_ENABLE_PCL)


add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC ${ProcessOR_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/.>")

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROCESSOR_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

add_definitions("-DSOFA_BUILD_PROCESSOR")

## Install rules for the library; CMake package configurations files
sofa_create_package(${PROJECT_NAME} ${PROCESSOR_VERSION} ${PROJECT_NAME} ${PROJECT_NAME})

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/" DESTINATION "include/processor"
  FILES_MATCHING PATTERN "*.h" PATTERN "*.inl")
