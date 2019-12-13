cmake_minimum_required(VERSION 3.1)

project(ImageProcessing_test)

set(LIBS SofaTest SofaGTestMain SofaCV ImageProcessing)

set(HEADER_FILES
)

set(SOURCE_FILES
 camera/common/CameraSettings_test.cpp
 common/DataSliderMgr_test.cpp
)

find_package(OpenSSL QUIET)
if (OpenSSL_FOUND)
    include_directories(${OPENSSL_INCLUDE_DIR})
    list(APPEND LIBS ${OPENSSL_LIBRARIES})
    list(APPEND HEADER_FILES  regression/Regression_test.h)
    list(APPEND SOURCE_FILES  regression/Regression_test.cpp)
endif (OpenSSL_FOUND)



add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${LIBS})

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
