cmake_minimum_required(VERSION 3.16)
project(SonyFocusStacker CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra -pthread)

set(CRSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/crsdk")
set(CRSDK_INCLUDE_DIR "${CRSDK_DIR}/include")
set(CRSDK_LIB_DIR "${CRSDK_DIR}/lib")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CRSDK_INCLUDE_DIR}
)

link_directories(${CRSDK_LIB_DIR})
set(CMAKE_INSTALL_RPATH "${CRSDK_LIB_DIR}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

add_executable(focus_stacker
    src/main.cpp
    src/StackingEngine.cpp
)

target_link_libraries(focus_stacker
    PRIVATE
    CrSDK
    pthread
)
