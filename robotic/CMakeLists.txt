include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					"${PROJECT_SOURCE_DIR}/base/inc"
					"${PROJECT_SOURCE_DIR}/base/matrix"
					"${PROJECT_SOURCE_DIR}/middleware/inc"
					"${PROJECT_SOURCE_DIR}/visualize/render"
					"${PROJECT_SOURCE_DIR}/collision/Broadphase"
		            )
add_library(robot STATIC robot.cpp trajectory.cpp )
