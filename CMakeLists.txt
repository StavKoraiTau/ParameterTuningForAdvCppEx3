cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(ALGO_COMMON
    AlgorithmRegistration.h
    SmartAlgorithm.h
    PathingGraph.h
    TileManager.h
    PathingGraph.cpp
    SmartAlgorithm.cpp
    TileManager.cpp
)
set(COMMON
    AlgorithmRegistrar.h
    AbstractAlgorithm.h
    BatteryMeter.h
    DirtSensor.h
    enums.h
    WallSensor.h
)

# Header files (relative to "include" directory)
set(HEADERS
    SmartAlgorithm2.h
)

# Source files (relative to "src" directory)
set(SOURCES
    SmartAlgorithm2.cpp
)

# Project setup
project(SmartAlgorithm2
    VERSION 1.0
    DESCRIPTION "Second algorithm in assignment 3 in the course Advanced Topics in Programming, by Asaf and Stav"
    LANGUAGES CXX)

list(TRANSFORM HEADERS PREPEND "include/")
list(TRANSFORM SOURCES PREPEND "src/")
list(TRANSFORM COMMON PREPEND "../../common/")
list(TRANSFORM ALGO_COMMON PREPEND "../algo_common/")
add_library("${PROJECT_NAME}" SHARED "${SOURCES}" "${HEADERS}" "${COMMON}" "${ALGO_COMMON}")

target_include_directories("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include/")
target_include_directories("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../")
target_include_directories("${PROJECT_NAME}" PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../")

if(DEFINED "TOP_LEVEL_COMPILE_ARGS")
    target_compile_options("${PROJECT_NAME}" PRIVATE "${TOP_LEVEL_COMPILE_ARGS}")
else()
    target_compile_options("${PROJECT_NAME}" PRIVATE -std=c++20 -Wall -Wextra -Werror -pedantic)
    target_compile_options("${PROJECT_NAME}" PRIVATE -O3)
endif()

if(DEFINED "TOP_LEVEL_LINKER_ARGS")
    target_link_options("${PROJECT_NAME}" PRIVATE "${TOP_LEVEL_LINKER_ARGS}")
else()
endif()

if(DEFINED "ALGO_OUTPUT_PATH")
    set_target_properties("${PROJECT_NAME}" PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${ALGO_OUTPUT_PATH}"
    )
endif()