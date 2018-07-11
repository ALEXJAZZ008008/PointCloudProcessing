PROJECT(PointCloudProcessing VERSION 0.0 LANGUAGES CXX)

add_library(PointCloudProcessing ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(PointCloudProcessing) 

target_sources(PointCloudProcessing 
	PRIVATE
	"${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessing.cpp"
    # "${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessing.h"
    ) 

install(TARGETS PointCloudProcessing DESTINATION lib)
