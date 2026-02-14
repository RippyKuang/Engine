#include "engine.hpp"

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <chrono>

struct PushConstantData
{
    glm::mat4 model;
};
struct UniformBufferObject
{
    glm::vec3 lightPos;
    float _pad0;
    glm::vec3 cameraPos;
    float _pad1;
    glm::mat4 view;
    glm::mat4 proj;
};

EngineApplication::EngineApplication()
{

    createUniformBuffers();
    createDescriptorSetLayout();
    createDescriptorPool();
    createPipelineLayout();

    pipeline = std::make_unique<PipeLine>(
        device,
        renderer.getSwapChainRenderPass(),
        pipelineLayout,
        "vert.spv",
        "frag.spv");
    swapChainExtent = renderer.getSwapChainExtent();
}

EngineApplication::~EngineApplication()
{
    for (size_t i = 0; i < SwapChain::MAX_FRAMES_IN_FLIGHT; i++)
    {
        vkDestroyBuffer(device.getDevice(), uniformBuffers[i], nullptr);
        vkFreeMemory(device.getDevice(), uniformBuffersMemory[i], nullptr);
    }
    vkDestroyDescriptorPool(device.getDevice(), descriptorPool, nullptr);
    vkDestroyDescriptorSetLayout(device.getDevice(), descriptorSetLayout, nullptr);
    daemon_running = false;
    daemon.join();
}
void EngineApplication::run()
{
    daemon_running = true;
    daemon = std::thread(&EngineApplication::daemon_run, this);
}
void EngineApplication::daemon_run()
{
    using namespace std;
    using namespace chrono;

    const glm::vec3 camera_pos = {0.5f, 0.5f, 0.5f};
    const glm::vec3 light_pos = {0.0f, 0.0f, 1.5f};
    UniformBufferObject ubo{};

    ubo.view = glm::lookAt(camera_pos, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float)swapChainExtent.height, 0.1f, 10.0f);
    ubo.proj[1][1] *= -1;
    ubo.cameraPos = camera_pos;
    ubo.lightPos = light_pos;

    while ((!window.shouldClose()) && daemon_running)
    {
        // auto start = std::chrono::steady_clock::now();
        glfwPollEvents();
        if (auto commandBuffer = renderer.beginFrame())
        {
            int frameIndex = renderer.getFrameIndex();
            memcpy(uniformBuffersMapped[frameIndex], &ubo, sizeof(ubo));
            renderer.beginSwapChainRenderPass(commandBuffer);
            pipeline->bind(commandBuffer);
            model->bind(commandBuffer);
            vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1,
                                    &descriptorSets[frameIndex], 0, nullptr);
            {
                // std::lock_guard<std::mutex> lock(m);
                for (int i = 0; i < objCount; i++)
                {

                    PushConstantData push{};
                    push.model = pose[i];

                    vkCmdPushConstants(commandBuffer,
                                       pipelineLayout,
                                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                       0,
                                       sizeof(PushConstantData),
                                       &push);
                    model->draw(commandBuffer, 36, 36 * i);
                }

                model->draw(commandBuffer, 6, 36 * objCount);
                model->draw(commandBuffer, 6, 36 * objCount + 6);
                model->draw(commandBuffer, 6, 36 * objCount + 12);
            }

            renderer.endSwapChainRenderPass(commandBuffer);
            renderer.endFrame();
        }
        // auto end = steady_clock::now();
        // auto duration = duration_cast<microseconds>(end - start);

        // cout << "花费了"
        //      << double(duration.count()) * microseconds::period::num / microseconds::period::den
        //      << "秒" << endl;
    }

    vkDeviceWaitIdle(device.getDevice());
}

void EngineApplication::update(std::vector<Engine::Matrix<float, 4, 4>> &mpose)
{
    for (int i = 0; i < mpose.size(); i++)
        pose[i] = glm::transpose(glm::make_mat4(mpose[i].data));
    for (int i = mpose.size(); i < objCount; i++)
        pose[i] = glm::mat4(1);
}
void EngineApplication::loadWorld(const std::vector<Engine::Link *> links)
{
    std::vector<Model::Vertex> vertices;
    std::vector<INDICES_TYPE> indices;
    objCount = links.size();
    pose.reserve(links.size());
    offset.reserve(links.size() + 1);
    offset[0] = 0;
    for (int i = 0; i < links.size(); i++)
    {
        std::vector<Model::Vertex> vert;
        std::vector<INDICES_TYPE> ind;
        parseLink(links[i], vert, ind, offset[i]);
        offset[i + 1] = offset[i] + vert.size();
        vertices.insert(vertices.end(), vert.begin(), vert.end());
        indices.insert(indices.end(), ind.begin(), ind.end());
    }
    // y axis
    int bias = vertices.size();
    vertices.push_back({{0, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    vertices.push_back({{0.0005, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    vertices.push_back({{0, 0.5, 0}, {0, 1, 0}, {0, 0, 1}});
    vertices.push_back({{0.0005, 0.5, 0}, {0, 1, 0}, {0, 0, 1}});
    indices.push_back(bias + 0);
    indices.push_back(bias + 1);
    indices.push_back(bias + 2);
    indices.push_back(bias + 1);
    indices.push_back(bias + 3);
    indices.push_back(bias + 2);

    // x axis
    bias = vertices.size();
    vertices.push_back({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}});
    vertices.push_back({{0, 0.0005, 0}, {1, 0, 0}, {0, 0, 1}});
    vertices.push_back({{0.5, 0, 0}, {1, 0, 0}, {0, 0, 1}});
    vertices.push_back({{0.5, +0.0005, 0}, {1, 0, 0}, {0, 0, 1}});
    indices.push_back(bias + 2);
    indices.push_back(bias + 1);
    indices.push_back(bias + 0);
    indices.push_back(bias + 2);
    indices.push_back(bias + 3);
    indices.push_back(bias + 1);

    // z axis
    bias = vertices.size();
    vertices.push_back({{0, 0, 0}, {0, 0, 1}, {1, 0, 0}});
    vertices.push_back({{0, 0.0005, 0}, {0, 0, 1}, {1, 0, 0}});
    vertices.push_back({{0, 0, 0.5}, {0, 0, 1}, {1, 0, 0}});
    vertices.push_back({{0, 0.0005, 0.5}, {0, 0, 1}, {1, 0, 0}});
    indices.push_back(bias + 0);
    indices.push_back(bias + 1);
    indices.push_back(bias + 2);
    indices.push_back(bias + 1);
    indices.push_back(bias + 3);
    indices.push_back(bias + 2);

    indexCount = indices.size();
    model = std::make_unique<Model>(device, vertices, indices);
}

void EngineApplication::createDescriptorSetLayout()
{
    VkDescriptorSetLayoutBinding uboLayoutBinding{};
    uboLayoutBinding.binding = 0;
    uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBinding.descriptorCount = 1;
    uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    uboLayoutBinding.pImmutableSamplers = nullptr;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &uboLayoutBinding;

    if (vkCreateDescriptorSetLayout(device.getDevice(), &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to create descriptor set layout");
    }
}

void EngineApplication::createPipelineLayout()
{
    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(PushConstantData);
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(device.getDevice(), &pipelineLayoutInfo, nullptr, &pipelineLayout) !=
        VK_SUCCESS)
    {
        throw std::runtime_error("failed to create pipeline layout!");
    }
}

void EngineApplication::createUniformBuffers()
{
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
    uniformBuffersMemory.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
    uniformBuffersMapped.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);

    for (size_t i = 0; i < SwapChain::MAX_FRAMES_IN_FLIGHT; i++)
    {
        device.createBuffer(bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                            uniformBuffers[i], uniformBuffersMemory[i]);
        vkMapMemory(device.getDevice(), uniformBuffersMemory[i], 0, bufferSize, 0, &uniformBuffersMapped[i]);
    }
}

void EngineApplication::createDescriptorPool()
{
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = static_cast<uint32_t>(SwapChain::MAX_FRAMES_IN_FLIGHT);

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = static_cast<uint32_t>(SwapChain::MAX_FRAMES_IN_FLIGHT);
    if (vkCreateDescriptorPool(device.getDevice(), &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to create descriptor pool!");
    }

    std::vector<VkDescriptorSetLayout> layouts(SwapChain::MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    allocInfo.descriptorSetCount = static_cast<uint32_t>(SwapChain::MAX_FRAMES_IN_FLIGHT);
    allocInfo.pSetLayouts = layouts.data();
    descriptorSets.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
    if (vkAllocateDescriptorSets(device.getDevice(), &allocInfo, descriptorSets.data()) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    for (size_t i = 0; i < SwapChain::MAX_FRAMES_IN_FLIGHT; i++)
    {
        VkDescriptorBufferInfo bufferInfo{};
        bufferInfo.buffer = uniformBuffers[i];
        bufferInfo.offset = 0;
        bufferInfo.range = sizeof(UniformBufferObject);

        VkWriteDescriptorSet descriptorWrite{};
        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        descriptorWrite.dstSet = descriptorSets[i];
        descriptorWrite.dstBinding = 0;
        descriptorWrite.dstArrayElement = 0;
        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        descriptorWrite.descriptorCount = 1;
        descriptorWrite.pBufferInfo = &bufferInfo;
        descriptorWrite.pImageInfo = nullptr; // Optional
        descriptorWrite.pTexelBufferView = nullptr;
        vkUpdateDescriptorSets(device.getDevice(), 1, &descriptorWrite, 0, nullptr);
    }
}

void EngineApplication::parseLink(Engine::Link *link, std::vector<Model::Vertex> &vertices, std::vector<INDICES_TYPE> &indices, uint32_t offset)
{
    std::vector<Engine::Vector3f> c = link->fget_corners();

    for (int i : {0, 1, 2, 3})
        vertices.push_back({{c[i][0], c[i][1], c[i][2]}, {1.0f, 1.0f, 1.0f}, {1, 0, 0}});

    for (int i : {4, 5, 6, 7})
        vertices.push_back({{c[i][0], c[i][1], c[i][2]}, {1.0f, 1.0f, 1.0f}, {-1, 0, 0}});

    for (int i : {0, 1, 4, 5})
        vertices.push_back({{c[i][0], c[i][1], c[i][2]}, {1.0f, 1.0f, 1.0f}, {0, -1, 0}});

    for (int i : {2, 3, 6, 7})
        vertices.push_back({{c[i][0], c[i][1], c[i][2]}, {1.0f, 1.0f, 1.0f}, {0, 1, 0}});

    for (int i : {0, 2, 4, 6})
        vertices.push_back({{c[i][0], c[i][1], c[i][2]}, {1.0f, 1.0f, 1.0f}, {0, 0, 1}});

    for (int i : {1, 3, 5, 7})
        vertices.push_back({{c[i][0], c[i][1], c[i][2]}, {1.0f, 1.0f, 1.0f}, {0, 0, -1}});

    indices = {
        // face 0
        0, 1, 2, 2, 1, 3,
        // face 1
        5, 4, 6, 7, 5, 6,
        // face 2
        8, 10, 9, 9, 10, 11,
        // face 3
        12, 13, 14, 13, 15, 14,
        // face 4
        16, 17, 18, 18, 17, 19,
        // face 5
        20, 22, 21, 21, 22, 23};
    for (int i = 0; i < indices.size(); i++)
    {
        indices[i] += offset;
    }
}