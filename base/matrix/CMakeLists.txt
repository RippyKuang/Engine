include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}"
		            )
add_library(matrix STATIC geometry.cpp)

