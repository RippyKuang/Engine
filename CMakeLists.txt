cmake_minimum_required(VERSION 3.10)
project(engine VERSION 1.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
find_package(PkgConfig REQUIRED)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
add_subdirectory(visualize)
add_subdirectory(base)
add_executable(engine main.cpp)

target_link_libraries(engine PUBLIC visual base)
target_include_directories(engine PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/visualize/inc"
                          "${PROJECT_SOURCE_DIR}/base/inc"
                          ${GTK3_INCLUDE_DIRS}
                          )