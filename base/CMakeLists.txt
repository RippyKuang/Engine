
include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					"${PROJECT_SOURCE_DIR}/robotic/inc"
		            )
add_library(base STATIC world.cpp geometry.cpp items.cpp camera.cpp)
