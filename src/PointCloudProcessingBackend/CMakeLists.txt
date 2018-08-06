PROJECT(PointCloudProcessingBackend VERSION 0.0 LANGUAGES CXX)

add_library(PointCloudProcessingBackend ${CMAKE_CURRENT_LIST_DIR})

find_library(FLANN_S_LIB NAMES libflann_cpp IMPORTED)
find_library(FLANN_CPP_S_LIB NAMES libflann_cpp_s IMPORTED)


# or flann_cpp_s.lib /usr/lib/x86_64-linux-gnu/flann_s.lib

target_link_libraries(${PROJECT_NAME} PointCloudProcessingObject ${FLANN_S_LIB} ${FLANN_CPP_S_LIB}  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})

target_sources(PointCloudProcessingBackend
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessingBackend.cpp"
    # "${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessingBackend.h"
    )

install(TARGETS PointCloudProcessingBackend DESTINATION lib)
