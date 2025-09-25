cmake_policy(SET CMP0048 NEW)

if (ORIN)
    MESSAGE("Compile for ORIN ... ")

    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE RAW_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(TOLOWER "${RAW_ARCH}" DETECTED_ARCH)
    message(STATUS "[System] uname -m = ${DETECTED_ARCH}")

    # 统一判断逻辑
    if(DETECTED_ARCH MATCHES "x86_64|i[3456]86|amd64")
        SET(CMAKE_CXX_COMPILER "/usr/bin/aarch64-linux-gnu-g++-12")
        SET(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc-12")
    endif()

    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR aarch64)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

    IF(NOT BOOSTER_THIRD_PARTY_INCLUDE_DIR)
        SET(BOOSTER_THIRD_PARTY_INCLUDE_DIR /opt/robosys/third_party_orin/include CACHE PATH "Booster third party include dir")
    ENDIF()

    IF(NOT BOOSTER_THIRD_PARTY_LIB_DIR)
        SET(BOOSTER_THIRD_PARTY_LIB_DIR /opt/robosys/third_party_orin/lib CACHE PATH "Booster third party lib dir")
    ENDIF()

    include_directories(${BOOSTER_THIRD_PARTY_INCLUDE_DIR})
    link_directories(${BOOSTER_THIRD_PARTY_LIB_DIR})
else()
    SET(CMAKE_CXX_STANDARD 17)
endif()

project(
  ${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

cmake_minimum_required(VERSION 3.15...3.27)

SET(LIB_BOOSTER_ROBOTICS_SDK_ROS2 booster_robotics_sdk_ros2 CACHE STRING "Booster Robotics SDK ROS2")

if(ORIN)
    set(PLATFORM_SUFFIX "_orin")
    message(STATUS "Architecture is: ORIN")

    if(NOT INSTALL_DIR)
        set(INSTALL_DIR /opt/robosys CACHE PATH "rs_sdk install dir")
    endif()

    set(LIB_INSTALL_PREFIX ${INSTALL_DIR}/${LIB_BOOSTER_ROBOTICS_SDK}${PLATFORM_SUFFIX})
else()
    set(LIB_INSTALL_PREFIX ${CMAKE_INSTALL_LIBDIR})
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)

include_directories(BEFORE $(PROJECT_SOURCE_DIR)/include)
include_directories(BEFORE $(PROJECT_SOURCE_DIR)/include/booster/idl/b1)
include_directories(BEFORE $(PROJECT_SOURCE_DIR)/internal_include)

file(GLOB_RECURSE BOOSTER_ROBOTICS_SDK_SOURCES 
    "src/*.cpp"
    "src/*.cxx"
    "src/*.ipp")

list(FILTER BOOSTER_ROBOTICS_SDK_SOURCES EXCLUDE REGEX "src/python/.*\\.cpp")

file(GLOB_RECURSE BOOSTER_ROBOTICS_SDK_HEADERS 
    "include/*.h"
    "include/*.hpp")
file(GLOB_RECURSE DEMO_TEST_SOURCES 
    "test/demo_test/*.cpp"
    "test/demo_test/*.cxx"
    "test/demo_test/*.ipp")

add_library(${LIB_BOOSTER_ROBOTICS_SDK} ${BOOSTER_ROBOTICS_SDK_SOURCES})

link_libraries(${LIB_BOOSTER_ROBOTICS_SDK} fastrtps fastcdr libfoonathan_memory-0.7.3.a)

