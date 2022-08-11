cmake_minimum_required(VERSION 3.10)
project(rstream VERSION 0.1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED core imgcodecs)
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)

# Fetch cli11 dependency
include(${PROJECT_SOURCE_DIR}/cmake/cli11.cmake)

# Fetch spdlog dependency
include(${PROJECT_SOURCE_DIR}/cmake/spdlog.cmake)

include_directories(
    include
    ${cli11_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
    ${spdlog_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/convert.cpp
    src/device.cpp
    src/frameset.cpp
    src/util.cpp)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${realsense2_LIBRARY})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

add_executable(rstream_cli src/bin/rstream_cli.cpp)
target_link_libraries(rstream_cli ${PROJECT_NAME} Threads::Threads)
set_property(TARGET rstream_cli PROPERTY CXX_STANDARD 17)

install(
    TARGETS ${PROJECT_NAME} rstream_cli
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# pkg-config
configure_file(pkg_config/${PROJECT_NAME}.pc.in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)