include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}"
					"${PROJECT_SOURCE_DIR}/base/matrix"
					"${PROJECT_SOURCE_DIR}/middleware/inc"
		            )
add_library(render STATIC raster.cpp)
