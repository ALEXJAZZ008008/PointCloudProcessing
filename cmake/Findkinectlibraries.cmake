# - Try to find kinectlibraries
# Once done, this will define
#
#  KinectLibraries_FOUND - system has kinectlibraries
#  KinectLibraries_INCLUDE_DIRS - the kinectlibraries include directories
#  KinectLibraries_LIBRARIES - link these to use kinectlibraries

include(LibFindMacros)

# Use pkg-config to get hints about paths
# libfind_pkg_check_modules(KinectLibraries_PKGCONF kinectlibraries)

# IF(NOT KINECTLIBRARIES_ROOT)
#   IF(EXISTS "/usr/include/kinectlibraries")
#     SET(KINECTLIBRARIES_ROOT "/usr")
#   ELSEIF(EXISTS "/usr/local/include/kinectlibraries")
#     SET(KINECTLIBRARIES_ROOT "/usr/local")
#   ELSE()
#     MESSAGE("KINECTLIBRARIES_ROOT not set. Continuing anyway..")
#   ENDIF()
# ENDIF()

# Include dir
find_path(KinectLibraries_INCLUDE_DIR
  NAMES kinectlibraries.h
  PATHS ${KINESTLIBRARIES_ROOT}/include/kinectlibraries ${KinectLibraries_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(KinectLibraries_LIBRARY
  NAMES kinectlibraries
  PATHS ${KINECTLIBRARIES_ROOT}/lib ${KinectLibraries_PKGCONF_LIBRARY_DIRS}
)

find_library(KinectLibrariesSync_LIBRARY
  NAMES kinectlibraries_sync
  PATHS ${KINECTLIBRARIES_ROOT}/lib ${KinectLibraries_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(KinectLibraries_PROCESS_INCLUDES KinectLibraries_INCLUDE_DIR KinectLibraries_INCLUDE_DIRS)
set(KinectLibraries_PROCESS_LIBS KinectLibrariesSync_LIBRARY KinectLibraries_LIBRARY KinectLibraries_LIBRARIES)
libfind_process(KinectLibraries)
 
