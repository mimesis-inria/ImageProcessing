cmake_minimum_required(VERSION 2.8.12)
project(ImageProcessing)

set(${PROJECT_NAME}_MAJOR_VERSION 2)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION})

find_package(SofaFramework REQUIRED)
find_package(SofaGui REQUIRED)
find_package(SofaQtQuickGUI QUIET)

find_package(OpenCV 3.0 REQUIRED)
find_package(SofaCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions("-std=c++11")

set (SOFACV_ENABLE_OPENCV_CONTRIB OFF CACHE BOOL "Enables OpenCV's non-free features hosted in opencv_contrib")

if (SOFACV_ENABLE_OPENCV_CONTRIB)
  add_definitions("-DSOFACV_OPENCV_CONTRIB_ENABLED")
endif ()

set(HEADER_FILES
  ImageProcessingPlugin.h

  common/ImageFilter.h

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

set(${PROJECT_NAME}_LIBRARIES SofaCore SofaSimulationCommon ${OpenCV_LIBRARIES} SofaCV)
set(${PROJECT_NAME}_SOURCES ${SOURCE_FILES} ${HEADER_FILES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set (SOFACV_NEW_GUI_ENABLED ON CACHE BOOL "Enables ImageFilter Widgets for runSofa2")

if((SofaQtQuickGUI_FOUND) AND (SOFACV_NEW_GUI_ENABLED))
  find_package(Qt5QuickControls2 REQUIRED)
  find_package(Qt5Quick REQUIRED)
  find_package(Qt5Qml REQUIRED)
  find_package(GLEW REQUIRED)

  add_definitions("-DSOFACV_NEW_GUI_ENABLED")
  add_definitions("-DSOFA_BUILD_${PROJECT_NAME}_PLUGIN")
  add_definitions("-DHAS_GLEW")

  list(APPEND SOURCE_FILES
    gui/ImageFilterDisplay.cpp
    gui/ImageFilterModel.cpp
    common/ImageFilter_newGUI.cpp
    )

  list(APPEND HEADER_FILES
    gui/ImageFilterDisplay.h
    gui/ImageFilterModel.h
    common/ImageFilter_newGUI.h
    )

  set(MOC_HEADER_FILES
    gui/ImageFilterDisplay.h
    gui/ImageFilterModel.h
    )

  set(QML_FILES
    gui/qml/SofaWidgets/ImageFilterWidget.qml
    )

  set(QRC_FILES
    gui/qml/ImageProcessing_qml.qrc
    )

  list (APPEND ${PROJECT_NAME}_LIBRARIES
    SofaQtQuickGUI
    ${GLEW_LIBRARIES}
    )

  # do not use "set(CMAKE_AUTOMOC ON)" since all the mocced files
  # will be compiled by a single compilation unit leading to a
  # "out of heap space" issue on MSVC
  qt5_wrap_cpp(MOC_FILES ${MOC_HEADER_FILES})
  qt5_add_resources(RESOURCE_FILES ${QRC_FILES})

  set(${PROJECT_NAME}_SOURCES
    ${SOURCE_FILES}
    ${HEADER_FILES}
    ${MOC_FILES}
    ${RESOURCE_FILES}
    ${QML_FILES}
    ${QRC_FILES}
    )

  # to fix a bug when one is compiling a debug version of the code
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DQT_NO_DEBUG)
  endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DQT_PLUGIN)
else ()
  list(APPEND HEADER_FILES
    common/DataSliderMgr.h
    common/DataSliderMgr.inl
    common/ImageFilter_oldGUI.h
    )

  list(APPEND SOURCE_FILES
    common/ImageFilter_oldGUI.cpp
    )
  set(${PROJECT_NAME}_SOURCES
    ${SOURCE_FILES}
    ${HEADER_FILES}
    )
endif()


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
