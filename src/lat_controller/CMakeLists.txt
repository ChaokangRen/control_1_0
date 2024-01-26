cmake_minimum_required(VERSION 3.0.2)
project(lat_controller)

find_package(SGLog REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/include/control
)
set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
## Build

add_library(${PROJECT_NAME} STATIC
    mpc_controller.cpp
    trajectory_gen.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    qpOASES
    common
)