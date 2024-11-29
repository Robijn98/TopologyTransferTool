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
include_directories(${CMAKE_SOURCE_DIR}/include/)
# Now we add our target executable and the file it is built from.
add_executable(Wrap)
target_sources(Wrap PRIVATE ${CMAKE_SOURCE_DIR}/src/main.cpp ${CMAKE_SOURCE_DIR}/src/Wrap.cpp include/Wrap.h 
               ${CMAKE_SOURCE_DIR}/src/Convert.cpp ${CMAKE_SOURCE_DIR}/include/Convert.h)


target_link_libraries(Wrap PRIVATE  fmt::fmt-header-only)

#################################################################################
# Testing code
#################################################################################

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()
add_executable(ConvertTests)

target_sources(ConvertTests PRIVATE ${CMAKE_SOURCE_DIR}/tests/CurveTests.cpp
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
                                ${CMAKE_SOURCE_DIR}/include/processor.h

)
target_link_libraries(ConvertTests PRIVATE GTest::gtest GTest::gtest_main )

gtest_discover_tests(ConvertTests)
