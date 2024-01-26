cmake_minimum_required(VERSION 3.0.2)
project(lon_controller)

find_package(SGLog REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/thirdparty/Eigen
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/include/control
)

## Build

add_library(${PROJECT_NAME} STATIC
    lon_controller.cpp
    pid_controller.cpp
    pid_state_machine.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${SGLog_LIBRARIES}
    common
)