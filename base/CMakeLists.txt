
include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
		            )
add_library(base STATIC world.cpp geometry.cpp)
