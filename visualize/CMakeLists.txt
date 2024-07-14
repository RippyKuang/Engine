find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
                     ${GTK3_INCLUDE_DIRS}
		            )
link_directories(${GTK3_LIBRARY_DIRS})
add_library(visual STATIC visualize.cpp )
target_link_libraries(visual PRIVATE ${GTK3_LIBRARIES})
