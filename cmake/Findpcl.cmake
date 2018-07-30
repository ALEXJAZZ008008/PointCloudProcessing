# - Try to find pcl
# Once done, this will define
#
#  PCL_FOUND - system has pcl
#  PCL_INCLUDE_DIRS - the pcl include directories
#  PCL_LIBRARIES - link these to use pcl

include(LibFindMacros)

# Use pkg-config to get hints about paths
# libfind_pkg_check_modules(PCL_PKGCONF pcl)

# IF(NOT PCL_ROOT)
#   IF(EXISTS "/usr/include/pcl")
#     SET(PCL_ROOT "/usr")
#   ELSEIF(EXISTS "/usr/local/include/pcl")
#     SET(PCL_ROOT "/usr/local")
#   ELSE()
#     MESSAGE("PCL_ROOT not set. Continuing anyway..")
#   ENDIF()
# ENDIF()

# Include dir
find_path(PCL_INCLUDE_DIR
  NAMES pcl.h
  PATHS ${PCL_ROOT}/include/pcl ${PCL_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(PCL_LIBRARY
  NAMES pcl
  PATHS ${PCL_ROOT}/lib ${PCL_PKGCONF_LIBRARY_DIRS}
)

find_library(PCLSync_LIBRARY
  NAMES pcl_sync
  PATHS ${PCL_ROOT}/lib ${PCL_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let pcl_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(PCL_PROCESS_INCLUDES PCL_INCLUDE_DIR PCL_INCLUDE_DIRS)
set(PCL_PROCESS_LIBS PCLSync_LIBRARY PCL_LIBRARY PCL_LIBRARIES)
libfind_process(PCL)
 
