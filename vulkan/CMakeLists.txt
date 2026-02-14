find_package(Vulkan REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

file(GLOB  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(vulkanapp STATIC ${SOURCES})


set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SHADER_OUT_DIR ${CMAKE_BINARY_DIR})

set(GRAPHICS_VERT_SHADER ${SHADER_DIR}/triangle.vert)
set(GRAPHICS_FRAG_SHADER ${SHADER_DIR}/triangle.frag)
set(GRAPHICS_SPIRV_VERT ${SHADER_OUT_DIR}/vert.spv)
set(GRAPHICS_SPIRV_FRAG ${SHADER_OUT_DIR}/frag.spv)

add_custom_command(
     OUTPUT ${GRAPHICS_SPIRV_VERT}
     COMMAND ${Vulkan_GLSLC_EXECUTABLE}  ${GRAPHICS_VERT_SHADER} -o ${GRAPHICS_SPIRV_VERT}
     DEPENDS ${GRAPHICS_VERT_SHADER}
)

add_custom_command(
     OUTPUT ${GRAPHICS_SPIRV_FRAG}
     COMMAND ${Vulkan_GLSLC_EXECUTABLE}  ${GRAPHICS_FRAG_SHADER} -o ${GRAPHICS_SPIRV_FRAG}
     DEPENDS ${GRAPHICS_FRAG_SHADER}
)

add_custom_target(CompileShaders ALL
    DEPENDS ${GRAPHICS_SPIRV_VERT} ${GRAPHICS_SPIRV_FRAG}
)

target_include_directories(vulkanapp PRIVATE .
                    "${PROJECT_SOURCE_DIR}/base/inc"
                    "${PROJECT_SOURCE_DIR}/visualize/render"
					"${PROJECT_SOURCE_DIR}/base/matrix")

target_link_libraries(vulkanapp PRIVATE
    Vulkan::Vulkan
    glfw
    glm::glm
)

