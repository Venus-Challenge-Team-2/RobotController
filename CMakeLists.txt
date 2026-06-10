cmake_minimum_required(VERSION 3.14)
project(webcam_test)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_ROOT "/home/student/libpynq-5EID0-2023-v0.3.0" CACHE PATH "project root")

set(LIBPYNQ_A "${PROJECT_ROOT}/build/lib/libpynq.a")

add_custom_command(
    OUTPUT ${LIBPYNQ_A}
    COMMAND make -C ${PROJECT_ROOT}
    COMMENT "Building libpynq..."
)

add_custom_target(libpynq_build DEPENDS ${LIBPYNQ_A})

add_library(pynq STATIC IMPORTED)
set_target_properties(pynq PROPERTIES
    IMPORTED_LOCATION ${LIBPYNQ_A}
    INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_ROOT}/library/;${PROJECT_ROOT}/platform/"
)
add_dependencies(pynq libpynq_build)

find_package(OpenCV QUIET COMPONENTS core imgproc videoio imgcodecs)

add_executable(main
    main.cpp
    mqtt.cpp
    single.cpp
    vl53l0x.cpp
    vl53l0x.h
    scan.cpp
    camera.cpp
    ntc_temperature.cpp
    ntc_temperature.h
    tcs3200.c
    tcs3200.h
)

target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(main PRIVATE
    pynq
    opencv_core
    opencv_imgproc
    opencv_videoio
    opencv_imgcodecs
    m          # libm — required by mqtt.c (sin, cos, atan2, sqrt, fabs)
)
