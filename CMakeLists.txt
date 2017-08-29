cmake_minimum_required(VERSION 2.8.12)
project(ProcessOR)

set(PROCESSOR_MAJOR_VERSION 0)
set(PROCESSOR_MINOR_VERSION 1)
set(PROCESSOR_VERSION ${PROCESSOR_MAJOR_VERSION}.${PROCESSOR_MINOR_VERSION})

find_package(SofaQtQuickGUI QUIET)
include_directories( . )

set (SOFAOR_ENABLE_OPENCV_CONTRIB OFF CACHE BOOL "Enables OpenCV's non-free features hosted in opencv_contrib")

if (SOFAOR_ENABLE_OPENCV_CONTRIB)
    add_definitions("-DSOFAOR_OPENCV_CONTRIB_ENABLED")
endif ()

set(HEADER_FILES
  initPlugin.h

  common/FrameViewer.h
  common/PCViewer.h
  common/ImageFilter.h
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
  imgproc/Fill.h

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
  initPlugin.cpp

  common/FrameViewer.cpp
  common/PCViewer.cpp
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
  imgproc/Fill.cpp

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

set(ProcessOR_LIBRARIES SofaCore SofaSimulationCommon SofaORCommon ${OpenCV_LIBRARIES})
set(ProcessOR_SOURCES ${ProcessOR_SOURCES} ${SOURCE_FILES} ${HEADER_FILES})
include_directories(SofaORCommon/ ProcessOR/)

set (SOFAOR_NEW_GUI_ENABLED ON CACHE BOOL "Enables ImageFilter Widgets for runSofa2")

if((SofaQtQuickGUI_FOUND) AND (SOFAOR_NEW_GUI_ENABLED))
    find_package(Qt5QuickControls2 REQUIRED)
    find_package(Qt5Quick REQUIRED)
    find_package(Qt5Qml REQUIRED)
    find_package(GLEW REQUIRED)

    add_definitions("-DSOFAOR_NEW_GUI_ENABLED")
    add_definitions("-DSOFA_BUILD_PROCESSOR_PLUGIN")
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
        gui/qml/ProcessOR_qml.qrc
        )

    list (APPEND ProcessOR_LIBRARIES
        SofaQtQuickGUI
        ${GLEW_LIBRARIES}
        )

    # do not use "set(CMAKE_AUTOMOC ON)" since all the mocced files
    # will be compiled by a single compilation unit leading to a
    # "out of heap space" issue on MSVC
    qt5_wrap_cpp(MOC_FILES ${MOC_HEADER_FILES})
    qt5_add_resources(RESOURCE_FILES ${QRC_FILES})

    set(ProcessOR_SOURCES
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
    set(ProcessOR_SOURCES
        ${SOURCE_FILES}
        ${HEADER_FILES}
        )
endif()


add_library(${PROJECT_NAME} SHARED ${ProcessOR_SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${ProcessOR_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/.>")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROCESSOR_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${PROCESSOR_PLUGIN_COMPILER_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

sofa_create_package(${PROJECT_NAME} ${PROCESSOR_VERSION} ${PROJECT_NAME} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        COMPONENT ${PROJECT_NAME}_libraries
#        EXPORT ${PROJECT_NAME}Targets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION "include/ProcessOR")

