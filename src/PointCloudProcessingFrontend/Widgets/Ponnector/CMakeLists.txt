PROJECT(Ponnector VERSION 0.0 LANGUAGES C CXX)

include_directories(${PCL_INCLUDE_DIR})

add_executable(${PROJECT_NAME} "${CMAKE_CURRENT_LIST_DIR}/main.cpp")

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets Qt5::Gui PointCloudProcessingBackend Logger)

target_sources(Ponnector
    PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/ponnector.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ponnector.ui"
    "${CMAKE_CURRENT_LIST_DIR}/ponnector.h"
    "${CMAKE_CURRENT_LIST_DIR}/ponnector_settings.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ponnector_settings.ui"
    "${CMAKE_CURRENT_LIST_DIR}/ponnector_settings.h")
