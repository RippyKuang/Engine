find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
add_subdirectory(render)
include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					"${CMAKE_CURRENT_SOURCE_DIR}/render"
					"${PROJECT_SOURCE_DIR}/base/inc"
					"${PROJECT_SOURCE_DIR}/base/matrix"
                     ${GTK3_INCLUDE_DIRS}
		            )
link_directories(${GTK3_LIBRARY_DIRS})
add_library(visual STATIC visualize.cpp)
target_link_libraries(visual PUBLIC ${GTK3_LIBRARIES} render)
