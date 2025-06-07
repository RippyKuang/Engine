
include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					"${CMAKE_CURRENT_SOURCE_DIR}/matrix"
					"${PROJECT_SOURCE_DIR}/robotic/inc"
					"${PROJECT_SOURCE_DIR}/middleware/inc"
					"${PROJECT_SOURCE_DIR}/visualize/render"
					"${PROJECT_SOURCE_DIR}/collision/Broadphase"
		            )
add_subdirectory(matrix)

add_library(base STATIC world.cpp  items.cpp camera.cpp )
target_link_libraries(base PUBLIC matrix)
