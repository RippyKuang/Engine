cmake_minimum_required(VERSION 3.10)
project(engine VERSION 1.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -g3 -O3")
find_package(PkgConfig REQUIRED)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
add_subdirectory(visualize)
add_subdirectory(base)
add_subdirectory(robotic)
add_subdirectory(middleware)
add_subdirectory(collision)
add_executable(engine main.cpp)
target_link_libraries(engine PUBLIC visual base robot middleware collision)

target_include_directories(engine PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/visualize/inc"
                          "${PROJECT_SOURCE_DIR}/visualize/render"
                          "${PROJECT_SOURCE_DIR}/base/inc"
                          "${PROJECT_SOURCE_DIR}/base/matrix"
                          "${PROJECT_SOURCE_DIR}/robotic/inc"
                          "${PROJECT_SOURCE_DIR}/collision/Broadphase"
                          "${PROJECT_SOURCE_DIR}/middleware/inc"
                          ${GTK3_INCLUDE_DIRS}
                          )
