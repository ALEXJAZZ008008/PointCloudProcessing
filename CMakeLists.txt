cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set (CMAKE_CXX_STANDARD 11)

# Set additional project information

PROJECT(PointCloudProcessing VERSION 0.0 LANGUAGES CXX)
set(COPYRIGHT "Copyright (c) 2018. All rights reserved.")

############################  Options: Build What?  ############################
option(PCL_DOWNLOAD "Download PCL" OFF)
option(PCL_LOCAL "Use local PCL" ON)

option(EIGEN_DOWNLOAD "Download Eigen" OFF)
option(EIGEN_LOCAL "Use local Eigen" ON)

option(BUILD_DOC "Build documentation" ON)
option(ENABLE_TESTING "Enable tests" ON)

############################  Options: Build How?  #############################
include(commons)
option_with_print(BUILD_SHARED_LIBS "Build internally built libraries as shared, not static" OFF)
option_with_default(CMAKE_BUILD_TYPE "Build type (Release or Debug)" Debug)
option_with_default(CMAKE_INSTALL_LIBDIR "Directory to which libraries installed" lib)

# gcc specific stuff
if (CMAKE_COMPILER_IS_GNUCC)
    add_definitions(-Wall -Wno-deprecated)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=gnu++0x -ffast-math")
endif()

if((${CMAKE_CXX_COMPILER_ID} MATCHES "AppleClang") OR (APPLE AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" AND
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
endif()

# For building the documentation only in Release mode.
if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")
    if (BUILD_DOC)
        find_package(Doxygen)
        if (DOXYGEN_FOUND)
            # set input and output files
            set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
            set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.in)

            # request to configure the file
            configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
            message("Doxygen build started")

            # note the option ALL which allows to build the docs together with the application
            add_custom_target( doc_doxygen ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM )
        else (DOXYGEN_FOUND)
            message("Doxygen need to be installed to generate the doxygen documentation")
        endif()
    endif()
endif()

if(NOT EIGEN_LOCAL)
	if (NOT EIGEN_DOWNLOAD)
		message(STATUS "${PROJECT_NAME} depends on EIGEN. You must either have it installed locally or download them.")
	endif()

else()
	set(EIGEN_DIR ""  CACHE PATH "Root path of EIGEN.")
endif()

# Then check if we should download.
if(EIGEN_DOWNLOAD)

    set(EIGEN_LOCAL "OFF")

    configure_file(${CMAKE_MODULE_PATH}/eigen.txt.in eigen-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/eigen-download")

    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/eigen-src/eigen-git-mirror/Eigen)
        add_dependencies(${PROJECT_NAME} eigen)

    set(EIGEN_FOUND ON)

endif()

# if(NOT PCL_LOCAL)
# 	if (NOT PCL_DOWNLOAD)
# 		message(STATUS "${PROJECT_NAME} depends on PCL. You must either have it installed locally or download them.")
# 	endif()

# else()
# 	set(PCL_DIR ""  CACHE PATH "Root path of PCL.")
# endif()

# if (PCL_DOWNLOAD)

# 	set(PCL_LOCAL "OFF")

# 	configure_file({CMAKE_MODULE_PATH}/pcl.txt.in pcl-download/CMakeLists.txt)
	
# endif()

include(src/CMakeLists.txt)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL "A flag showing that CMake has configured at least once.")
