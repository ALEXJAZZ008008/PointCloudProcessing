# Copyright 2018
# University of Hull

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

PROJECT(PointCloudProcessing VERSION 0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

############################  Options: Build What?  ############################

option(DOWNLOAD_PCL "Downlaod the PCL libraries" OFF)
option(DOWNLOAD_KinectLibraries "Downlaod the Kinect Libraries" OFF)
option(DOWNLOAD_Cereal "Downlaod the Cereal library" OFF)

option(LOCAL_PCL "Use Local PCL libraries" OFF)
option(LOCAL_KINECTLIBRARIES "Use Local KinectLibraries" OFF)
option(LOCAL_CEREAL "Use Local Cereal library" OFF)

option(BUILD_DOC "Build documentation" ON)

############################  Options: Build How?  #############################

include(commons)

option_with_print(BUILD_SHARED_LIBS "Build internally built libraries as shared, not static" OFF)
option_with_default(CMAKE_BUILD_TYPE "Build type (Release or Debug)" Debug)
option_with_default(CMAKE_INSTALL_LIBDIR "Directory to which libraries installed" lib)

if(CMAKE_COMPILER_IS_GNUCC)

    add_definitions(-Wall -Wno-deprecated)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=gnu++0x -ffast-math")

endif()

if((${CMAKE_CXX_COMPILER_ID} MATCHES "AppleClang") OR (APPLE AND ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" AND ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")

endif()

# We need the PCL libraries, either locally installed
# or we will download them from git.

if(NOT LOCAL_PCL)
    if(NOT DOWNLOAD_PCL)

        message(STATUS "${PROJECT_NAME} depends on PCL. You must either have it installed locally or download them.")

    endif()
else()

    set(PCL_ROOT "" CACHE PATH "Root path of PCL.")
    find_package(PCL)

endif()

if(NOT LOCAL_KINECTLIBRARIES)
    if(NOT DOWNLOAD_KinectLibraries)

        message(STATUS "${PROJECT_NAME} depends on kinectlibraries. You must either have it installed locally or download it.")

    endif()
else()

    set(KinectLibraries_ROOT "" CACHE PATH "Root path of kinectlibraries.")
    find_package(kinectlibraries)

endif()

if(NOT LOCAL_CEREAL)
    if(NOT DOWNLOAD_Cereal)

        message(STATUS "${PROJECT_NAME} depends on cereal. You must either have it installed locally or download it.")

    endif()
else()

    set(Cereal_ROOT "" CACHE PATH "Root path of cereal.")
    find_package(cereal)

endif()

# Then check if we should download.

if(DOWNLOAD_PCL)

    set(LOCAL_PCL "OFF")

    configure_file(${CMAKE_MODULE_PATH}/pcl.txt.in pcl-download/CMakeLists.txt)

    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pcl-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pcl-download")

    add_subdirectory("${CMAKE_BINARY_DIR}/pcl-src" "${CMAKE_BINARY_DIR}/pcl-build")

    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/pcl-src/include)

    set(PCL_LIB_DIR "${CMAKE_BINARY_DIR}/lib")

    if(WIN32)

        set(prefix "")
        set(suffix ".lib")

    elseif(APPLE)

        set(prefix "lib")
        set(suffix ".a")

    else()

        set(prefix "lib")
        set(suffix ".a")

    endif()

    set(PCL_LIBRARY "${PCL_LIB_DIR}/${prefix}pcl${suffix}")
    set(PCL_FOUND ON)

endif()

if(DOWNLOAD_KinectLibraries)

    set(LOCAL_KINECTLIBRARIES "OFF")

    configure_file(${CMAKE_MODULE_PATH}/kinectlibraries.txt.in kinectlibraries-download/CMakeLists.txt)

    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/kinectlibraries-download" )
    execute_process(COMMAND "${CMAKE_COMMAND}" --build . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/kinectlibraries-download")

    add_subdirectory("${CMAKE_BINARY_DIR}/kinectlibraries-src" "${CMAKE_BINARY_DIR}/kinectlibraries-build")

    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/kinectlibraries-src/include)

    set(KinectLibraries_LIB_DIR "${CMAKE_BINARY_DIR}/lib")

    if(WIN32)

        set(prefix "")
        set(suffix ".lib")

    elseif(APPLE)

        set(prefix "lib")
        set(suffix ".a")

    else()

        set(prefix "lib")
        set(suffix ".a")

    endif()

    set(KinectLibraries_LIBRARIES "${Cereal_LIB_DIR}/${prefix}kinectlibraries${suffix}")
    set(KinectLibraries_FOUND ON)

endif()

if(DOWNLOAD_Cereal)

    set(LOCAL_CEREAL "OFF")

    configure_file(${CMAKE_MODULE_PATH}/cereal.txt.in cereal-download/CMakeLists.txt)

    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/cereal-download" )
    execute_process(COMMAND "${CMAKE_COMMAND}" --build . WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/cereal-download")

    add_subdirectory("${CMAKE_BINARY_DIR}/cereal-src" "${CMAKE_BINARY_DIR}/cereal-build")

    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/cereal-src/include)

    set(Cereal_LIB_DIR "${CMAKE_BINARY_DIR}/lib")

    if(WIN32)

        set(prefix "")
        set(suffix ".lib")

    elseif(APPLE)

        set(prefix "lib")
        set(suffix ".a")

    else()

        set(prefix "lib")
        set(suffix ".a")

    endif()

    set(Cereal_LIBRARY "${Cereal_LIB_DIR}/${prefix}cereal${suffix}")
    set(Cereal_FOUND ON)

endif()

if(APPLE AND EXISTS /usr/local/opt/qt5)

    # Homebrew installs Qt5 (up to at least 5.9.1) in
    # /usr/local/qt5, ensure it can be found by CMake since
    # it is not in the default /usr/local prefix.

    list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt5")

endif()

# Find the Qt components

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# Threads are used in the LibFreenect sync library and
# we might use them too, in order to acquire data without
# freezing the system.

find_package(Threads REQUIRED)

# For building the documentation only in Release mode.
if(CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")
    if(BUILD_DOC)

        find_package(Doxygen)

        if(DOXYGEN_FOUND)

            # set input and output files

            set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
            set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

            # request to configure the file

            configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

            message("Doxygen build started")

            # note the option ALL which allows to build the docs together with the application

            add_custom_target(doc_doxygen
                ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM)

        else(DOXYGEN_FOUND)

            message("Doxygen need to be installed to generate the doxygen documentation")

        endif()
    endif()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(src/include)

if(PCL_FOUND AND KinectLibraries_FOUND AND Cereal_FOUND)

    include(src/CMakeLists.txt)

endif()

set(CONFIGURED_ONCE TRUE CACHE INTERNAL "A flag showing that CMake has configured at least once.")
