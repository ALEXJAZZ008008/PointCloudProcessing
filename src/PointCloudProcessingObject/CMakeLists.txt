PROJECT(PointCloudProcessingObject VERSION 0.0 LANGUAGES CXX)

add_library(PointCloudProcessingObject ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(PointCloudProcessingObject) 

target_sources(PointCloudProcessingObject 
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessingObject.cpp")

install(TARGETS PointCloudProcessingObject DESTINATION lib)
