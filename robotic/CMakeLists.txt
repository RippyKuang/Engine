include_directories("${PROJECT_BINARY_DIR}"
		            "${CMAKE_CURRENT_SOURCE_DIR}/inc"
					"${PROJECT_SOURCE_DIR}/base/inc"
					"${PROJECT_SOURCE_DIR}/middleware/inc"
		            )
add_library(robot STATIC robot.cpp)
