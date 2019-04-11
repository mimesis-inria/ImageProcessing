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
  src/ImageProcessingPlugin.h

  src/camera/common/CameraSettings.h
  src/camera/common/StereoSettings.h
  src/camera/common/CalibratedCamera.h
  src/camera/common/ImageRectifier.h
  src/camera/common/ProjectPoints.h
  src/camera/common/FeatureTriangulator.h

  src/camera/calib/CalibrateCamera.h
  src/camera/calib/SolvePnP.h
  src/camera/calib/CalibrateStereo.h
  src/camera/calib/CalibLoader.h
  src/camera/calib/CalibExporter.h
  src/camera/calib/FindChessboardCorners.h

  src/camera/control/CameraTrajectory.h
  src/camera/control/TrajectoryAround.h
  src/camera/control/LinesOfSightConstraintManager.h
  src/camera/control/LinesOfSightConstraintManager.inl
  src/camera/control/CameraController.h
  src/camera/control/Icosphere.h

  src/imgproc/CannyFilter.h
  src/imgproc/SobelFilter.h
  src/imgproc/AdaptiveThreshold.h
  src/imgproc/SimpleThreshold.h
  src/imgproc/MorphologyEx.h
  src/imgproc/ConvertTo.h
  src/imgproc/ConvertTo.inl
  src/imgproc/CvtColor.h
  src/imgproc/TemplateMatcher.h
  src/imgproc/AddWeighted.h
  src/imgproc/Flip.h
  src/imgproc/InRange.h
  src/imgproc/CopyTo.h
  src/imgproc/Resize.h
  src/imgproc/Crop.h
  src/imgproc/Fill.h
  src/imgproc/MinMaxLoc.h

  src/features/Detectors.h
  src/features/Matchers.h
  src/features/FeatureDetector.h
  src/features/DescriptorMatcher.h
  src/features/MatchingConstraints.h
  src/features/PointPicker2D.h
  src/features/Segmenter2D.h
  src/features/OpticalFlow.h
  src/features/FeatureColorExtractor.cpp

  src/utils/PointVectorConverter.h
  src/utils/PointVectorConverter.inl
  src/utils/PointVec2Keypoint.h
  src/utils/Vector2SSVector.h
  src/utils/Vector2SSVector.inl
  src/utils/NegateVector.h
  src/utils/OrthoProj.h
  src/utils/AddCam.h
  )

set(SOURCE_FILES
  src/ImageProcessingPlugin.cpp

  src/camera/common/CameraSettings.cpp
  src/camera/common/StereoSettings.cpp
  src/camera/common/CalibratedCamera.cpp
  src/camera/common/ImageRectifier.cpp
  src/camera/common/ProjectPoints.cpp
  src/camera/common/FeatureTriangulator.cpp

  src/camera/calib/CalibrateCamera.cpp
  src/camera/calib/SolvePnP.cpp
  src/camera/calib/CalibrateStereo.cpp
  src/camera/calib/CalibLoader.cpp
  src/camera/calib/CalibExporter.cpp
  src/camera/calib/FindChessboardCorners.cpp

  src/camera/control/CameraTrajectory.cpp
  src/camera/control/TrajectoryAround.cpp
  src/camera/control/LinesOfSightConstraintManager.cpp
  src/camera/control/CameraController.cpp
  src/camera/control/Icosphere.cpp

  src/imgproc/CannyFilter.cpp
  src/imgproc/SobelFilter.cpp
  src/imgproc/AdaptiveThreshold.cpp
  src/imgproc/SimpleThreshold.cpp
  src/imgproc/MorphologyEx.cpp
  src/imgproc/ConvertTo.cpp
  src/imgproc/CvtColor.cpp
  src/imgproc/TemplateMatcher.cpp
  src/imgproc/AddWeighted.cpp
  src/imgproc/Flip.cpp
  src/imgproc/InRange.cpp
  src/imgproc/CopyTo.cpp
  src/imgproc/Resize.cpp
  src/imgproc/Crop.cpp
  src/imgproc/Fill.cpp
  src/imgproc/MinMaxLoc.cpp

  src/features/Detectors.cpp
  src/features/Matchers.cpp
  src/features/FeatureDetector.cpp
  src/features/DescriptorMatcher.cpp
  src/features/MatchingConstraints.cpp
  src/features/PointPicker2D.cpp
  src/features/Segmenter2D.cpp
  src/features/OpticalFlow.cpp
  src/features/FeatureColorExtractor.cpp

  src/utils/PointVectorConverter.cpp
  src/utils/PointVec2Keypoint.cpp
  src/utils/Vector2SSVector.cpp
  src/utils/NegateVector.cpp
  src/utils/OrthoProj.cpp
  src/utils/AddCam.h
  )

set(${PROJECT_NAME}_LIBRARIES SofaCore SofaSimulationCommon ${OpenCV_LIBRARIES} SofaCV)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${PROJECT_NAME} SHARED ${SOURCES_FILES} ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC -D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\")
target_compile_options(${PROJECT_NAME} PUBLIC -D${PROJECT_NAME}_REGRESSION_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/examples/\")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

sofa_generate_package(NAME ${PROJECT_NAME} VERSION ${${PROJECT_NAME}_VERSION} TARGETS ${PROJECT_NAME} INCLUDE_INSTALL_DIR ${PROJECT_NAME} INCLUDE_SOURCE_DIR "src/${PROJECT_NAME}")

if(SOFA_BUILD_TESTS)
    find_package(SofaTest QUIET)
    if(SofaTest_FOUND)
        add_subdirectory(ImageProcessing_test)
    endif()
endif()
