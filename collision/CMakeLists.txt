include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					"${PROJECT_SOURCE_DIR}/base/matrix"
					"${PROJECT_SOURCE_DIR}/collision/Broadphase"
		            )
add_library(collision STATIC Broadphase/obb.cpp)
