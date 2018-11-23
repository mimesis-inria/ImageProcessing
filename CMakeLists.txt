cmake_minimum_required(VERSION 2.8.12)
project(ImageProcessing)

#cmake_policy(SET CMP0053 OLD)
#cmake_policy(SET CMP0054 OLD)
#cmake_policy(SET CMP0017 NEW)

set(${PROJECT_NAME}_MAJOR_VERSION 2)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION})

find_package(SofaFramework REQUIRED)
find_package(SofaGui REQUIRED)

find_package(OpenCV 3.0 REQUIRED)
find_package(SofaCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions("-std=c++11")

include(CheckIncludeFileCXX)
check_include_file_cxx(opencv2/xfeatures2d.hpp HAVE_OPENCV_CONTRIB)
if(HAVE_OPENCV_CONTRIB)
  message("found opencv contrib")
    add_definitions(-DSOFACV_OPENCV_CONTRIB_ENABLED)
endif(HAVE_OPENCV_CONTRIB)

set(HEADER_FILES
  ImageProcessingPlugin.h

  camera/common/CameraSettings.h
  camera/common/StereoSettings.h
  camera/common/CalibratedCamera.h
  camera/common/ImageRectifier.h
  camera/common/ProjectPoints.h
  camera/common/FeatureTriangulator.h

  camera/calib/CalibrateCamera.h
  camera/calib/SolvePnP.h
  camera/calib/CalibrateStereo.h
  camera/calib/CalibLoader.h
  camera/calib/CalibExporter.h
  camera/calib/FindChessboardCorners.h

  camera/control/CameraTrajectory.h
  camera/control/TrajectoryAround.h
  camera/control/LinesOfSightConstraintManager.h
  camera/control/LinesOfSightConstraintManager.inl
  camera/control/CameraController.h
  camera/control/Icosphere.h

  imgproc/CannyFilter.h
  imgproc/SobelFilter.h
  imgproc/AdaptiveThreshold.h
  imgproc/SimpleThreshold.h
  imgproc/MorphologyEx.h
  imgproc/ConvertTo.h
  imgproc/ConvertTo.inl
  imgproc/CvtColor.h
  imgproc/TemplateMatcher.h
  imgproc/AddWeighted.h
  imgproc/Flip.h
  imgproc/InRange.h
  imgproc/CopyTo.h
  imgproc/Resize.h
  imgproc/Crop.h
  imgproc/Fill.h
  imgproc/MinMaxLoc.h

  features/Detectors.h
  features/Matchers.h
  features/FeatureDetector.h
  features/DescriptorMatcher.h
  features/MatchingConstraints.h
  features/PointPicker2D.h
  features/Segmenter2D.h
  features/OpticalFlow.h
  features/FeatureColorExtractor.cpp

  utils/PointVectorConverter.h
  utils/PointVectorConverter.inl
  utils/PointVec2Keypoint.h
  utils/Vector2SSVector.h
  utils/Vector2SSVector.inl
  utils/NegateVector.h
  utils/OrthoProj.h
  )

set(SOURCE_FILES
  ImageProcessingPlugin.cpp

  camera/common/CameraSettings.cpp
  camera/common/StereoSettings.cpp
  camera/common/CalibratedCamera.cpp
  camera/common/ImageRectifier.cpp
  camera/common/ProjectPoints.cpp
  camera/common/FeatureTriangulator.cpp

  camera/calib/CalibrateCamera.cpp
  camera/calib/SolvePnP.cpp
  camera/calib/CalibrateStereo.cpp
  camera/calib/CalibLoader.cpp
  camera/calib/CalibExporter.cpp
  camera/calib/FindChessboardCorners.cpp

  camera/control/CameraTrajectory.cpp
  camera/control/TrajectoryAround.cpp
  camera/control/LinesOfSightConstraintManager.cpp
  camera/control/CameraController.cpp
  camera/control/Icosphere.cpp

  imgproc/CannyFilter.cpp
  imgproc/SobelFilter.cpp
  imgproc/AdaptiveThreshold.cpp
  imgproc/SimpleThreshold.cpp
  imgproc/MorphologyEx.cpp
  imgproc/ConvertTo.cpp
  imgproc/CvtColor.cpp
  imgproc/TemplateMatcher.cpp
  imgproc/AddWeighted.cpp
  imgproc/Flip.cpp
  imgproc/InRange.cpp
  imgproc/CopyTo.cpp
  imgproc/Resize.cpp
  imgproc/Crop.cpp
  imgproc/Fill.cpp
  imgproc/MinMaxLoc.cpp

  features/Detectors.cpp
  features/Matchers.cpp
  features/FeatureDetector.cpp
  features/DescriptorMatcher.cpp
  features/MatchingConstraints.cpp
  features/PointPicker2D.cpp
  features/Segmenter2D.cpp
  features/OpticalFlow.cpp
  features/FeatureColorExtractor.cpp

  utils/PointVectorConverter.cpp
  utils/PointVec2Keypoint.cpp
  utils/Vector2SSVector.cpp
  utils/NegateVector.cpp
  utils/OrthoProj.cpp
  )

set(${PROJECT_NAME}_SOURCES ${SOURCE_FILES} ${HEADER_FILES})
set(${PROJECT_NAME}_LIBRARIES SofaCore SofaSimulationCommon ${OpenCV_LIBRARIES} SofaCV)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})

target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC -D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\")
target_compile_options(${PROJECT_NAME} PUBLIC -D${PROJECT_NAME}_REGRESSION_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/examples/\")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

sofa_create_package(${PROJECT_NAME} ${${PROJECT_NAME}_VERSION} ${PROJECT_NAME} ${PROJECT_NAME})

if(SOFA_BUILD_TESTS)
    find_package(SofaTest QUIET)
    if(SofaTest_FOUND)
        add_subdirectory(ImageProcessing_test)
    endif()
endif()
