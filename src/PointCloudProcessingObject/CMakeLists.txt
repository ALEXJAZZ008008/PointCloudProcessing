PROJECT(PointCloudProcessingObject VERSION 0.0 LANGUAGES CXX)

add_library(PointCloudProcessingObject ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${PROJECT_NAME} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARY_DIRS})

target_sources(PointCloudProcessingObject
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessingObject.cpp")

install(TARGETS PointCloudProcessingObject DESTINATION lib)
