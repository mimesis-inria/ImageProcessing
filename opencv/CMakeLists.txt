cmake_minimum_required(VERSION 2.8.12)
project(SofaOROpenCV)

set(SOFAOROPENCV_MAJOR_VERSION 0)
set(SOFAOROPENCV_MINOR_VERSION 1)
set(SOFAOROPENCV_VERSION ${SOFAOROPENCV_MAJOR_VERSION}.${SOFAOROPENCV_MINOR_VERSION})

set(HEADER_FILES
  initPlugin.h
  core/ImageFilter.h
  core/ImageExporter.h

  utils/PointVectorConverter.h
  utils/PointVectorConverter.inl
  utils/PointVec2Keypoint.h
  utils/Vector2SSVector.h
  utils/Vector2SSVector.inl
  utils/NegateVector.h
#  utils/ConversionFunctor.h
#  utils/ConversionFunctor.inl

  features/Detectors.h
  features/Matchers.h
  features/FeatureDetector.h
  features/DescriptorMatcher.h
  features/FeatureTriangulator.h
  features/MatchingConstraints.h
  features/OpticalFlow.h

  imgproc/CannyFilter.h
  imgproc/SobelFilter.h
  imgproc/ImageRectifier.h
  imgproc/Segmenter2D.h
  imgproc/PointPicker2D.h
  imgproc/SimpleThreshold.h
  imgproc/AdaptiveThreshold.h
  imgproc/MorphologyEx.h

  calib/CalibLoader.h
#  calib/ComputeRT.h
  calib/CameraSettings.h
  calib/StereoSettings.h
#  calib/CameraProjection.h
#  calib/CameraProjectionMat.h
#  calib/CameraModelview.h
#  calib/CamGizmo.h

  calib/CalibrateCamera.h
  calib/CalibrateStereo.h
  calib/SolvePnP.h
  calib/CalibratedCamera.h
  #  calib/CalibStereo.h
  calib/FrameViewer.h
  calib/PCViewer.h

#  PointPicker3D.h
  )

set(SOURCE_FILES
  initPlugin.cpp

  core/ImageFilter.cpp
  core/ImageExporter.cpp

  utils/PointVectorConverter.cpp
  utils/PointVec2Keypoint.cpp
  utils/Vector2SSVector.cpp
  utils/NegateVector.cpp
#  utils/ConversionFunctor.cpp

  features/Detectors.cpp
  features/Matchers.cpp
  features/FeatureDetector.cpp
  features/DescriptorMatcher.cpp
  features/FeatureTriangulator.cpp
  features/MatchingConstraints.cpp
  features/OpticalFlow.cpp

  imgproc/CannyFilter.cpp
  imgproc/SobelFilter.cpp
  imgproc/ImageRectifier.cpp
  imgproc/Segmenter2D.cpp
  imgproc/PointPicker2D.cpp
  imgproc/SimpleThreshold.cpp
  imgproc/AdaptiveThreshold.cpp
  imgproc/MorphologyEx.cpp

  calib/CalibLoader.cpp
#  calib/ComputeRT.cpp
  calib/CameraSettings.cpp
  calib/StereoSettings.cpp
#  calib/CameraProjection.cpp
#  calib/CameraProjectionMat.cpp
#  calib/CameraModelview.cpp
#  calib/CamGizmo.cpp

  calib/CalibrateCamera.cpp
  calib/CalibrateStereo.cpp
  calib/SolvePnP.cpp
  calib/CalibratedCamera.cpp
  #  calib/CalibStereo.cpp
  calib/FrameViewer.cpp
  calib/PCViewer.cpp

#  PointPicker3D.cpp
  )

find_package(SofaQtQuickGUI QUIET)

if(SofaQtQuickGUI_FOUND)
    find_package(Qt5QuickControls2 REQUIRED)
    find_package(Qt5Quick REQUIRED)
    find_package(Qt5Qml REQUIRED)

    include_directories("/home/bruno/dev/sofaqtquick/applications-dev/plugins")

    list(APPEND SOURCE_FILES
	core/gui/ImageViewerDisplay.cpp
	core/gui/ImageViewerModel.cpp
    )
    list(APPEND HEADER_FILES
	core/gui/ImageViewerDisplay.h
	core/gui/ImageViewerModel.h
    )

    set(MOC_HEADER_FILES
	core/gui/ImageViewerDisplay.h
	core/gui/ImageViewerModel.h
    )

    # set(MOC_FILES
    #     ${XRAYRENDERING_PLUGIN_SOURCE_DIR}/gui/XRayViewerDisplay.h
    # )

    set(QML_FILES
	# ${XRAYRENDERING_PLUGIN_SOURCE_DIR}/gui/qml/qmldir
	core/gui/qml/SofaWidgets/ImageViewerWidget.qml
    )

    set(QRC_FILES
	core/gui/qml/imagerenderer_qml.qrc
    )

    # to fix a bug when one is compiling a debug version of the code
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
	add_definitions(-DQT_NO_DEBUG)
    endif()
    add_definitions(-DQT_PLUGIN)

    set(CMAKE_AUTOMOC ON)
    qt5_wrap_cpp(MOC_FILES ${MOC_HEADER_FILES}) # do not use "set(CMAKE_AUTOMOC ON)" since all the mocced files will be compiled by a single compilation unit leading to a "out of heap space" issue on MSVC
    qt5_add_resources(RESOURCE_FILES ${QRC_FILES})

    set(OpenCV_LIBRARIES SofaQtQuickGUI ${OpenCV_LIBRARIES})
endif()

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
