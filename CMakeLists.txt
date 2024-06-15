cmake_minimum_required(VERSION 3.22)
project(Boids)

find_package(Git)
message(STATUS "GIT_EXECUTABLE='${GIT_EXECUTABLE}'")

### CMake config ###
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 
set(CMAKE_CXX_STANDARD 20)


if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS YES)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}")

add_library(project_warnings INTERFACE)
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
include(cmake/CompilerFlags.cmake)
include(cmake/ExternalTools.cmake)


add_compile_options()

add_subdirectory(src)

set(CMAKE_BINARY_DIR ${PROJECT_SOURCE_DIR})
set_target_properties(CDTDemo 
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)