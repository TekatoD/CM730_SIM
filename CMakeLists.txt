cmake_minimum_required(VERSION 3.8)
project(CM_730_EMU)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/modules/")

find_package(VREP REQUIRED)

include_directories(${VREP_INCLUDE})
include_directories(${VREP_REMOTE})

set(SOURCE_FILES
        CM730.cpp
        CM730.h
        MX28.cpp
        MX28.h
        JointData.cpp
        JointData.h
        main.cpp)

add_executable(CM_730_EMU ${SOURCE_FILES})
target_link_libraries(CM_730_EMU pthread ${VREP_LIBRARIES})