cmake_minimum_required(VERSION 3.0.2)
project(common)


find_package(PkgConfig QUIET)
pkg_check_modules(PC_DEPEND
    eigen3
    sglog
    sgtime
    jsoncpp
)

message("depend inc: ${PC_DEPEND_INCLUDE_DIRS}")
message("depend lib: ${PC_DEPEND_LIBRARIES}")
message("depend lib_dir: ${PC_DEPEND_LIBRARY_DIRS}")

include_directories(
    ${CMAKE_SOURCE_DIR}/include/control
    ${PC_DEPEND_INCLUDE_DIRS}
)

link_directories(${PC_DEPEND_LIBRARY_DIRS})

## Build

add_library(${PROJECT_NAME} STATIC
    json_parser.cpp
    common.cpp
    digital_filter.cpp
    digital_filter_coefficients.cpp
    interpolation_2d.cpp
    trajectory_analyzer.cpp
    runge_kutta.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PC_DEPEND_LIBRARIES}
)