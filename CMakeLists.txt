# We will always try to use a version > 3.1 if avaliable
cmake_minimum_required(VERSION 3.2)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()
# name of the project It is best to use something different from the exe name
project(Wrap_build)
find_package(fmt CONFIG REQUIRED)

# Here we set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)
# add include paths
include_directories(include)
# Now we add our target executable and the file it is built from.
add_executable(Wrap)
target_sources(Wrap PRIVATE ${CMAKE_SOURCE_DIR}/src/main.cpp ${CMAKE_SOURCE_DIR}/src/Wrap.cpp include/Wrap.h)
target_link_libraries(Particle PRIVATE  fmt::fmt-header-only)

#################################################################################
# Testing code
#################################################################################

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()
add_executable(ParticleTests)
target_sources(WrapTests PRIVATE ${CMAKE_SOURCE_DIR}/tests/WrapTests.cpp
                                ${CMAKE_SOURCE_DIR}/src/Wrap.cpp
                                ${CMAKE_SOURCE_DIR}/include/Wrap.h

)
target_link_libraries(WrapTests PRIVATE GTest::gtest GTest::gtest_main )
gtest_discover_tests(WrapTests)
