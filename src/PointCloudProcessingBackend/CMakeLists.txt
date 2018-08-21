PROJECT(PointCloudProcessingBackend VERSION 0.0 LANGUAGES CXX)

add_library(PointCloudProcessingBackend ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${PROJECT_NAME} PointCloudProcessingObject) 

target_sources(PointCloudProcessingBackend 
	PRIVATE
	"${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessingBackend.cpp"
    # "${CMAKE_CURRENT_LIST_DIR}/PointCloudProcessingBackend.h"
    ) 

install(TARGETS PointCloudProcessingBackend DESTINATION lib)
