cmake_minimum_required(VERSION 2.8.12)
project(ProcessOR)

set(PROCESSOR_MAJOR_VERSION 0)
set(PROCESSOR_MINOR_VERSION 1)
set(PROCESSOR_VERSION ${PROCESSOR_MAJOR_VERSION}.${PROCESSOR_MINOR_VERSION})

set(HEADER_FILES
  initPlugin.h
  )

set(SOURCE_FILES
  initPlugin.cpp
  )

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

sofa_add_plugin(opencv SofaOROpenCV)
