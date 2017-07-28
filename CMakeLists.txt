cmake_minimum_required(VERSION 2.8.12)
project(ProcessOR)

set(PROCESSOR_MAJOR_VERSION 0)
set(PROCESSOR_MINOR_VERSION 1)
set(PROCESSOR_VERSION ${PROCESSOR_MAJOR_VERSION}.${PROCESSOR_MINOR_VERSION})

set (SOFAOR_ENABLE_OPENCV_CONTRIB OFF CACHE BOOL "Enables OpenCV's non-free features hosted in opencv_contrib")

if (SOFAOR_ENABLE_OPENCV_CONTRIB)
    add_definitions("-DSOFAOR_OPENCV_CONTRIB_ENABLED")
endif ()

set(HEADER_FILES
  initPlugin.h

  common/FrameViewer.h
  common/PCViewer.h
  common/ImageFilter.h
  common/DataSliderMgr.h
  common/DataSliderMgr.inl
  common/ImageExporter.h

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

  imgproc/CannyFilter.h
  imgproc/SobelFilter.h
  imgproc/AdaptiveThreshold.h
  imgproc/SimpleThreshold.h
  imgproc/MorphologyEx.h
  imgproc/ConvertTo.h
  imgproc/ConvertTo.inl
  imgproc/CvtColor.h
  imgproc/ImageMask.h
  imgproc/TemplateMatcher.h
  imgproc/AddWeighted.h
  imgproc/Flip.h
  imgproc/InRange.h
  imgproc/CopyTo.h
  imgproc/Resize.h

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
#  utils/ConversionFunctor.h
#  utils/ConversionFunctor.inl
  )

set(SOURCE_FILES
  initPlugin.cpp

  common/FrameViewer.cpp
  common/PCViewer.cpp
  common/ImageFilter.cpp
  common/ImageExporter.cpp

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

  imgproc/CannyFilter.cpp
  imgproc/SobelFilter.cpp
  imgproc/AdaptiveThreshold.cpp
  imgproc/SimpleThreshold.cpp
  imgproc/MorphologyEx.cpp
  imgproc/ConvertTo.cpp
  imgproc/CvtColor.cpp
  imgproc/ImageMask.cpp
  imgproc/TemplateMatcher.cpp
  imgproc/AddWeighted.cpp
  imgproc/Flip.cpp
  imgproc/InRange.cpp
  imgproc/CopyTo.cpp
  imgproc/Resize.cpp

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
  #  utils/ConversionFunctor.cpp
  )

include_directories(ProcessOR/)

set(ProcessOR_LIBRARIES SofaCore SofaSimulationCommon SofaORCommon ${OpenCV_LIBRARIES})

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

