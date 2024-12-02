# We will always try to use a version > 3.1 if available
cmake_minimum_required(VERSION 3.2)


if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

# Name of the project
project(Wrap_build)

# Find required packages
find_package(fmt CONFIG REQUIRED)
find_package(CGAL REQUIRED) 

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add include paths
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/include/)

# Main executable target
add_executable(Wrap)
target_sources(Wrap PRIVATE ${CMAKE_SOURCE_DIR}/src/main.cpp 
                            ${CMAKE_SOURCE_DIR}/src/mesh.cpp 
                            include/mesh.h 
                            ${CMAKE_SOURCE_DIR}/src/curve.cpp 
                            ${CMAKE_SOURCE_DIR}/include/curve.h)

# Link libraries to the main executable
target_link_libraries(Wrap PRIVATE fmt::fmt-header-only CGAL::CGAL) 

#################################################################################
# Testing code
#################################################################################

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()

# Test target
add_executable(meshTests)

target_sources(meshTests PRIVATE ${CMAKE_SOURCE_DIR}/tests/curveTests.cpp
                                    ${CMAKE_SOURCE_DIR}/tests/meshTests.cpp
                                    ${CMAKE_SOURCE_DIR}/tests/outputTests.cpp
                                    ${CMAKE_SOURCE_DIR}/tests/processorTests.cpp

                                    ${CMAKE_SOURCE_DIR}/src/curve.cpp
                                    ${CMAKE_SOURCE_DIR}/src/mesh.cpp
                                    ${CMAKE_SOURCE_DIR}/src/output.cpp
                                    ${CMAKE_SOURCE_DIR}/src/processor.cpp

                                    ${CMAKE_SOURCE_DIR}/include/curve.h
                                    ${CMAKE_SOURCE_DIR}/include/mesh.h
                                    ${CMAKE_SOURCE_DIR}/include/output.h
                                    ${CMAKE_SOURCE_DIR}/include/processor.h)

# Link libraries to the test target
target_link_libraries(meshTests PRIVATE GTest::gtest 
                                         GTest::gtest_main 
                                         CGAL::CGAL) 

# Discover tests
gtest_discover_tests(meshTests)
