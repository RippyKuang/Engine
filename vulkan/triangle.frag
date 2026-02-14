
#version 450

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;

layout(location = 0) out vec4 outColor;


layout(binding=0)uniform UniformBufferObject
{
    vec3 lightPos;
    vec3 cameraPos;

    mat4 view;
    mat4 proj;
}ubo;


const vec3  kd = vec3(0.8);   
const vec3  ks = vec3(0.5);   
const float shininess = 32.0;
const vec3 lightColor = vec3(1,1,1);

void main()
{
  
    vec3 N = normalize(vNormal);
    vec3 L = normalize(ubo.lightPos - vWorldPos);
    vec3 V = normalize(ubo.cameraPos - vWorldPos);
    vec3 H = normalize(L + V);

  
    float dist = length(ubo.lightPos - vWorldPos);
    float attenuation = 1.0 / (1.0 + 0.09 * dist + 0.032 * dist * dist);

  
    float NdotL = max(dot(N, L), 0.0);
    vec3 diffuse = kd * NdotL* lightColor ;

  
    float NdotH = max(dot(N, H), 0.0);
    vec3 specular = ks * lightColor * pow(NdotH, shininess);

   
    vec3 ambient = 0.5 * kd;

    vec3 color = (ambient + diffuse + specular) * attenuation;
    outColor = vec4(color, 1.0);
}
