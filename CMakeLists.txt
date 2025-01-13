# We will always try to use a version > 3.1 if available
cmake_minimum_required(VERSION 3.12)


if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
   set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

# Name of the project
project(Wrap_build)
set(TargetName Wrap)

find_package(NGL CONFIG REQUIRED)

set(CMAKE_AUTOMOC ON)
#
# Find the QtWidgets library
find_package(Qt6 COMPONENTS OpenGL Widgets QUIET )
if ( Qt6_FOUND )
    message("Found Qt6 Using that")
else()
    message("Found Qt5 Using that")
    find_package(Qt5 COMPONENTS OpenGL Widgets REQUIRED)
endif()


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Find required packages
find_package(fmt CONFIG REQUIRED)
find_package(CGAL REQUIRED) 
message(STATUS "Found CGAL version ${CGAL_VERSION}")

add_executable(Wrap)

# Add include paths
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(include $ENV{HOME}/NGL/include)

# Main executable target
target_sources(Wrap PRIVATE ${CMAKE_SOURCE_DIR}/src/main.cpp 
                            ${CMAKE_SOURCE_DIR}/src/mesh.cpp 
                            ${CMAKE_SOURCE_DIR}/include/mesh.h 
                            ${CMAKE_SOURCE_DIR}/src/curve.cpp 
                            ${CMAKE_SOURCE_DIR}/include/curve.h
                            ${CMAKE_SOURCE_DIR}/include/meshUtility.h
                            ${CMAKE_SOURCE_DIR}/src/meshUtility.cpp
                            ${CMAKE_SOURCE_DIR}/include/output.h
                            ${CMAKE_SOURCE_DIR}/src/output.cpp
                            ${PROJECT_SOURCE_DIR}/src/NGLScene.cpp  
                            ${PROJECT_SOURCE_DIR}/src/NGLSceneMouseControls.cpp  
                            ${PROJECT_SOURCE_DIR}/include/NGLScene.h
                            ${PROJECT_SOURCE_DIR}/src/NGLScene.cpp  
                            ${PROJECT_SOURCE_DIR}/src/NGLSceneMouseControls.cpp  
                            ${PROJECT_SOURCE_DIR}/include/NGLScene.h)

# Link libraries to the main executable
target_link_libraries(Wrap PRIVATE fmt::fmt-header-only CGAL::CGAL) 
target_link_libraries(${TargetName} PRIVATE Qt::Widgets Qt::OpenGL NGL)




add_custom_target(${TargetName}CopyShaders ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    $<TARGET_FILE_DIR:${TargetName}>/shaders
)

#################################################################################
# Testing code
#################################################################################

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()

# Test target
add_executable(allTests)

target_sources(allTests PRIVATE ${CMAKE_SOURCE_DIR}/tests/curveTests.cpp
                                    ${CMAKE_SOURCE_DIR}/tests/meshTests.cpp
                                    ${CMAKE_SOURCE_DIR}/tests/meshUtilityTests.cpp
                                    ${CMAKE_SOURCE_DIR}/tests/outputTests.cpp

                                    ${CMAKE_SOURCE_DIR}/src/curve.cpp
                                    ${CMAKE_SOURCE_DIR}/src/mesh.cpp
                                    ${CMAKE_SOURCE_DIR}/src/meshUtility.cpp
                                    ${CMAKE_SOURCE_DIR}/src/output.cpp

                                    ${CMAKE_SOURCE_DIR}/include/curve.h
                                    ${CMAKE_SOURCE_DIR}/include/mesh.h
                                    ${CMAKE_SOURCE_DIR}/include/meshUtility.h
                                    ${CMAKE_SOURCE_DIR}/include/output.h)

# Link libraries to the test target
target_link_libraries(allTests PRIVATE GTest::gtest 
                                         GTest::gtest_main 
                                         CGAL::CGAL) 

# Discover tests
gtest_discover_tests(allTests)



if(EXISTS ${CMAKE_SOURCE_DIR}/test_files)
    # Directory exists, proceed with copying files
    message(STATUS "DIRECTORY 'test_files' EXISTS. Proceeding with setup.")

    # Custom target to create the directory and copy test files
    add_custom_target(
        CopyTestfiles ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/files
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_SOURCE_DIR}/test_files
                ${CMAKE_BINARY_DIR}/files
        COMMENT "Copying test files to ${CMAKE_BINARY_DIR}/files"
    )
else()
    # Directory does not exist, skip setup
    message(WARNING "DIRECTORY 'test_files' DOES NOT EXIST. Skipping setup.")
endif()

# Add the dependency for curveTests
add_dependencies(allTests CopyTestfiles)
