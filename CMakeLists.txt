cmake_minimum_required(VERSION 2.8.12)
project(ImageProcessing)

#cmake_policy(SET CMP0053 OLD)
#cmake_policy(SET CMP0054 OLD)
#cmake_policy(SET CMP0017 NEW)

set(${PROJECT_NAME}_MAJOR_VERSION 2)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION})

find_package(SofaFramework REQUIRED)
find_package(SofaGeneral REQUIRED)
find_package(SofaGui REQUIRED)

find_package(OpenCV REQUIRED)
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

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/ImageProcessing/")

set(HEADER_FILES
  src/ImageProcessing/ImageProcessingPlugin.h

  src/ImageProcessing/camera/common/CameraSettings.h
  src/ImageProcessing/camera/common/StereoSettings.h
  src/ImageProcessing/camera/common/CalibratedCamera.h
  src/ImageProcessing/camera/common/ImageRectifier.h
  src/ImageProcessing/camera/common/ProjectPoints.h
  src/ImageProcessing/camera/common/FeatureTriangulator.h

  src/ImageProcessing/camera/calib/CalibrateCamera.h
  src/ImageProcessing/camera/calib/SolvePnP.h
  src/ImageProcessing/camera/calib/CalibrateStereo.h
  src/ImageProcessing/camera/calib/CalibLoader.h
  src/ImageProcessing/camera/calib/CalibExporter.h
  src/ImageProcessing/camera/calib/FindChessboardCorners.h

  src/ImageProcessing/camera/control/CameraTrajectory.h
  src/ImageProcessing/camera/control/TrajectoryAround.h
  src/ImageProcessing/camera/control/LinesOfSightConstraintManager.h
  src/ImageProcessing/camera/control/LinesOfSightConstraintManager.inl
  src/ImageProcessing/camera/control/CameraController.h
  src/ImageProcessing/camera/control/Icosphere.h

  src/ImageProcessing/imgproc/CannyFilter.h
  src/ImageProcessing/imgproc/SobelFilter.h
  src/ImageProcessing/imgproc/AdaptiveThreshold.h
  src/ImageProcessing/imgproc/SimpleThreshold.h
  src/ImageProcessing/imgproc/MorphologyEx.h
  src/ImageProcessing/imgproc/ConvertTo.h
  src/ImageProcessing/imgproc/ConvertTo.inl
  src/ImageProcessing/imgproc/CvtColor.h
  src/ImageProcessing/imgproc/TemplateMatcher.h
  src/ImageProcessing/imgproc/AddWeighted.h
  src/ImageProcessing/imgproc/Flip.h
  src/ImageProcessing/imgproc/InRange.h
  src/ImageProcessing/imgproc/CopyTo.h
  src/ImageProcessing/imgproc/Resize.h
  src/ImageProcessing/imgproc/Crop.h
  src/ImageProcessing/imgproc/Fill.h
  src/ImageProcessing/imgproc/MinMaxLoc.h

  src/ImageProcessing/features/Detectors.h
  src/ImageProcessing/features/Matchers.h
  src/ImageProcessing/features/FeatureDetector.h
  src/ImageProcessing/features/DescriptorMatcher.h
  src/ImageProcessing/features/MatchingConstraints.h
  src/ImageProcessing/features/PointPicker2D.h
  src/ImageProcessing/features/Segmenter2D.h
  src/ImageProcessing/features/OpticalFlow.h
  src/ImageProcessing/features/FeatureColorExtractor.cpp

  src/ImageProcessing/utils/PointVectorConverter.h
  src/ImageProcessing/utils/PointVectorConverter.inl
  src/ImageProcessing/utils/PointVec2Keypoint.h
  src/ImageProcessing/utils/Vector2SSVector.h
  src/ImageProcessing/utils/Vector2SSVector.inl
  src/ImageProcessing/utils/NegateVector.h
  src/ImageProcessing/utils/OrthoProj.h
  src/ImageProcessing/utils/AddCam.h
  )

set(SOURCE_FILES
  src/ImageProcessing/ImageProcessingPlugin.cpp

  src/ImageProcessing/camera/common/CameraSettings.cpp
  src/ImageProcessing/camera/common/StereoSettings.cpp
  src/ImageProcessing/camera/common/CalibratedCamera.cpp
  src/ImageProcessing/camera/common/ImageRectifier.cpp
  src/ImageProcessing/camera/common/ProjectPoints.cpp
  src/ImageProcessing/camera/common/FeatureTriangulator.cpp

  src/ImageProcessing/camera/calib/CalibrateCamera.cpp
  src/ImageProcessing/camera/calib/SolvePnP.cpp
  src/ImageProcessing/camera/calib/CalibrateStereo.cpp
  src/ImageProcessing/camera/calib/CalibLoader.cpp
  src/ImageProcessing/camera/calib/CalibExporter.cpp
  src/ImageProcessing/camera/calib/FindChessboardCorners.cpp

  src/ImageProcessing/camera/control/CameraTrajectory.cpp
  src/ImageProcessing/camera/control/TrajectoryAround.cpp
  src/ImageProcessing/camera/control/LinesOfSightConstraintManager.cpp
  src/ImageProcessing/camera/control/CameraController.cpp
  src/ImageProcessing/camera/control/Icosphere.cpp

  src/ImageProcessing/imgproc/CannyFilter.cpp
  src/ImageProcessing/imgproc/SobelFilter.cpp
  src/ImageProcessing/imgproc/AdaptiveThreshold.cpp
  src/ImageProcessing/imgproc/SimpleThreshold.cpp
  src/ImageProcessing/imgproc/MorphologyEx.cpp
  src/ImageProcessing/imgproc/ConvertTo.cpp
  src/ImageProcessing/imgproc/CvtColor.cpp
  src/ImageProcessing/imgproc/TemplateMatcher.cpp
  src/ImageProcessing/imgproc/AddWeighted.cpp
  src/ImageProcessing/imgproc/Flip.cpp
  src/ImageProcessing/imgproc/InRange.cpp
  src/ImageProcessing/imgproc/CopyTo.cpp
  src/ImageProcessing/imgproc/Resize.cpp
  src/ImageProcessing/imgproc/Crop.cpp
  src/ImageProcessing/imgproc/Fill.cpp
  src/ImageProcessing/imgproc/MinMaxLoc.cpp

  src/ImageProcessing/features/Detectors.cpp
  src/ImageProcessing/features/Matchers.cpp
  src/ImageProcessing/features/FeatureDetector.cpp
  src/ImageProcessing/features/DescriptorMatcher.cpp
  src/ImageProcessing/features/MatchingConstraints.cpp
  src/ImageProcessing/features/PointPicker2D.cpp
  src/ImageProcessing/features/Segmenter2D.cpp
  src/ImageProcessing/features/OpticalFlow.cpp
  src/ImageProcessing/features/FeatureColorExtractor.cpp

  src/ImageProcessing/utils/PointVectorConverter.cpp
  src/ImageProcessing/utils/PointVec2Keypoint.cpp
  src/ImageProcessing/utils/Vector2SSVector.cpp
  src/ImageProcessing/utils/NegateVector.cpp
  src/ImageProcessing/utils/OrthoProj.cpp
  src/ImageProcessing/utils/AddCam.h
  )

set(${PROJECT_NAME}_LIBRARIES SofaCore SofaSimulationCommon SofaConstraint Sofa.GL ${OpenCV_LIBRARIES} SofaCV)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

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
