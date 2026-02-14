#version 450

layout(binding=0)uniform UniformBufferObject
{
    vec3 lightPos;
    vec3 cameraPos;

    mat4 view;
    mat4 proj;
}ubo;

layout(location=0)in vec3 position;
layout(location=1)in vec3 color;
layout(location=2)in vec3 normal;

layout(location=0)out vec3 vPosition;
layout(location=1)out vec3 vNormal;
layout(location=2)out vec3 vcolor;

layout(push_constant)uniform Push
{
    mat4 model;
}push;

void main()
{
    vec4 worldPos = push.model * vec4(position, 1.0);
    mat3 normalMat = transpose(inverse(mat3(push.model)));
    vNormal = normalMat * normal;
    vPosition = worldPos.xyz;
    vcolor = color;
    gl_Position = ubo.proj * ubo.view * worldPos;
    
}